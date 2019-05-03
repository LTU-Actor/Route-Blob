#include <ltu_actor_route_blob/BlobConfig.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <ros/ros.h>
#include <string>

class Blob {
public:
    Blob();

    bool hasSub();
    bool isEnabled();
    void shutdown();
    void startup();

private:
    ros::NodeHandle                 nh;
    image_transport::ImageTransport it;
    ros::Publisher                  twist_pub;

    void dynConfigCB(ltu_actor_route_blob::BlobConfig &config, uint32_t level);
    void dashcamCB(const sensor_msgs::ImageConstPtr &);

    std::string camera_topic;

    bool      enabled = false;
    bool      show_video = true;
    bool      ready = false;

    // Main camera input
    image_transport::Subscriber image_sub;

    // Image publishers for debugging
    void debug_img_pub_color(image_transport::Publisher &pub,
                             const cv::Mat &             img);
    void debug_img_pub_bw(image_transport::Publisher &pub, const cv::Mat &img);
    image_transport::Publisher debug_pub_blob;
    image_transport::Publisher debug_pub_dilated;
    image_transport::Publisher debug_pub_lines;
    image_transport::Publisher debug_pub_edges;
    image_transport::Publisher debug_pub_result;

    // Server for run-time parameter adjustment
    dynamic_reconfigure::Server<ltu_actor_route_blob::BlobConfig> dyn_server;

    cv::Mat    current_image;
    std::mutex current_image_mutex;
    cv::Mat    getCurrentImage()
    {
        std::lock_guard<std::mutex> lock(current_image_mutex);
        ready = false;
        return current_image;
    }

    struct {
        // Dynamic reconfigure parameters
        ltu_actor_route_blob::BlobConfig dynamic;

        // Put other config paremeters (nav_controller) below
    } config;

    void  find_edges(cv::Mat &in, cv::Mat &out);
    float blob_adjust(const cv::Mat &edges, cv::Mat &debug_display);
};

Blob::Blob()
  : nh{"~"},
    it(nh)
{
    if (!nh.getParam("input", camera_topic))
    {
        ROS_ERROR_STREAM("No camera topic passed to /actor_input/dashcam");
        throw std::invalid_argument("Bad camera topic");
    }

    // TODO: read cam topic
    image_sub = it.subscribe(camera_topic, 1, &Blob::dashcamCB, this);

    debug_pub_blob    = it.advertise("debug_blob", 1);
    debug_pub_dilated = it.advertise("debug_dilated", 1);
    debug_pub_lines   = it.advertise("debug_lines", 1);
    debug_pub_edges   = it.advertise("debug_edges", 1);
    debug_pub_result  = it.advertise("debug_result", 1);

    dynamic_reconfigure::Server<ltu_actor_route_blob::BlobConfig>::CallbackType
                          dynConfigCB;
    ltu_actor_route_blob::BlobConfig default_config;
    dynConfigCB = boost::bind(&Blob::dynConfigCB, this, _1, _2);
    dyn_server.setCallback(dynConfigCB);
    dyn_server.getConfigDefault(default_config);
    dynConfigCB(default_config, 0);
}

void Blob::dynConfigCB(ltu_actor_route_blob::BlobConfig &newconfig, uint32_t level)
{
    // Mutex not needed. individual values will be atomic by x86 architecture.
    config.dynamic = newconfig;
}

void Blob::dashcamCB(const sensor_msgs::ImageConstPtr &msg)
{
    //ROS_ERROR_STREAM("GOT IMAGE");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //ROS_ERROR_STREAM("RUNNING");

    cv::Mat              input;
    cv::Mat              edges;
    cv::Mat              hsv;
    cv::Mat              display;
    std::vector<cv::Mat> channels(3);
    input = getCurrentImage();

    cv::cvtColor(input, hsv, CV_BGR2HSV);
    cv::split(hsv, channels);
    // Color
    //channels[2] -= channels[1];
    cv::medianBlur(channels[2], channels[2],
                    config.dynamic.enhance_blur * 2 + 1);
    cv::merge(channels, display);
    cv::cvtColor(display, display, CV_HSV2BGR);

    if (config.dynamic.edge_method == 0)
        find_edges(display, edges);
    else if (config.dynamic.edge_method == 1)
        find_edges(channels[2], edges);
    else if (config.dynamic.edge_method == 2)
    {
        cv::adaptiveThreshold(
            channels[2], edges, 255,
            config.dynamic.adap_use_gauss ? cv::ADAPTIVE_THRESH_MEAN_C
                                            : cv::ADAPTIVE_THRESH_MEAN_C,
            cv::THRESH_BINARY, config.dynamic.adap_block_size * 2 + 1,
            config.dynamic.adap_c);
    }
    else
    {
        cv::Laplacian(channels[2], edges, -1,
                        config.dynamic.lapla_ksize * 2 + 1);
        cv::Sobel(edges, edges, -1, config.dynamic.sobel_xorder,
                    config.dynamic.sobel_yorder,
                    config.dynamic.sobel_ksize * 2 + 1);
    }

    float turn;
    if (config.dynamic.lines_enable)
    {
        cv::Mat lines_mat = cv::Mat::zeros(edges.size(), edges.type());
        std::vector<cv::Vec4i> lines;
        cv::Rect               rect =
            cv::Rect(0, config.dynamic.lines_top * edges.rows, edges.cols,
                        edges.rows - config.dynamic.lines_top * edges.rows);

        cv::HoughLinesP(edges(rect), lines, config.dynamic.lines_rho,
                        0.01745329251, config.dynamic.lines_thresh,
                        config.dynamic.lines_min_len,
                        config.dynamic.lines_max_gap);

        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];

            float diffx = l[0] - l[2];
            float diffy = l[1] - l[3];

            float slope = diffy / diffx;

            if (std::abs(slope) < config.dynamic.lines_min_slope) continue;

            diffx *= 5;
            diffy *= 5;

            l[0] -= diffx;
            l[1] -= diffy;
            l[2] += diffx;
            l[3] += diffy;

            cv::line(
                lines_mat,
                cv::Point(l[0],
                            l[1] + config.dynamic.lines_top * edges.rows),
                cv::Point(l[2],
                            l[3] + config.dynamic.lines_top * edges.rows),
                255, 5);
        }

        turn = blob_adjust(lines_mat, display);

        debug_img_pub_bw(debug_pub_lines, lines_mat);
    }
    else
    {
        turn = blob_adjust(edges, display);
    }

    debug_img_pub_bw(debug_pub_edges, edges);
    debug_img_pub_color(debug_pub_result, display);

    cv::waitKey(3);

    geometry_msgs::Twist twist;
    twist.linear.x  = config.dynamic.drive_speed;
    twist.angular.z = -config.dynamic.blob_mult * turn;

    if (!config.dynamic.enable_drive)
    {
        twist.linear.x  = 0;
        twist.angular.z = 0;
    }
    else if (!config.dynamic.enable_forward)
    {
        twist.linear.x = 0;
    }

    twist_pub.publish(twist);

}

void Blob::debug_img_pub_color(image_transport::Publisher &pub,
                                        const cv::Mat &             img)
{
    if (pub.getNumSubscribers() > 0)
        pub.publish(
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg());
}

void Blob::debug_img_pub_bw(image_transport::Publisher &pub,
                                     const cv::Mat &             img)
{
    if (pub.getNumSubscribers() > 0)
        pub.publish(
            cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg());
}


void Blob::find_edges(cv::Mat &in, cv::Mat &out)
{
    cv::Canny(in, out, config.dynamic.canny_lower_thresh,
              config.dynamic.canny_upper_thresh,
              config.dynamic.canny_aperture_size * 2 + 1);
}

float Blob::blob_adjust(const cv::Mat &edges, cv::Mat &debug_display)
{
    cv::Mat dilated;
    cv::Mat display = cv::Mat::zeros(edges.size(), CV_8UC3);

    const int dilation_size  = config.dynamic.blob_dilation_size;
    cv::Mat   dilate_element = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(dilation_size, dilation_size));

    cv::dilate(edges, dilated, dilate_element);

    if (config.dynamic.blob_median_blur_size > 0)
        cv::medianBlur(dilated, dilated, config.dynamic.blob_median_blur_size * 2 + 1);

    struct Point {
        float x;
        float y;
    };

    std::vector<Point> points;
    points.reserve(config.dynamic.blob_num_points + 1);

    for (float theta = 0; theta <= M_PI;
         theta += M_PI / config.dynamic.blob_num_points)
    {
        Point p;
        p.x = config.dynamic.blob_x;
        p.y = config.dynamic.blob_y;

        float diffx = std::cos(theta) * .01;
        float diffy = -1 * std::sin(theta) * .01;

        while (dilated.at<uint8_t>(p.y * dilated.rows, p.x * dilated.cols)
               < config.dynamic.blob_num_points)
        {
            p.x += diffx;
            p.y += diffy;

            float top_y = 1 - config.dynamic.blob_max_p_y;
            if (p.y > 1 || p.y < top_y || p.x > 1 || p.x < 0)
            {
                if (p.x > 1) p.x = 1;
                if (p.x < 0) p.x = 0;
                if (p.y > 1) p.y = 1;
                if (p.y < top_y) p.y = top_y;
                break;
            }
        }

        cv::circle(display, cv::Point(p.x * display.cols, p.y * display.rows),
                   5, cv::Scalar(255, 0, 0), -1);
        cv::circle(debug_display,
                   cv::Point(p.x * display.cols, p.y * display.rows), 5,
                   cv::Scalar(255, 0, 0), -1);

        points.push_back(p);
    }

    Point center_p = {(float)config.dynamic.blob_x, (float)config.dynamic.blob_y};
    Point center_a = {0, 0};

    for (size_t i = 0; i < points.size(); i++)
    {
        Point &p     = points[i];
        float  diffx = center_p.x - p.x;
        float  diffy = center_p.y - p.y;

        center_a.x += p.x;
        center_a.y += p.y;

        float length = std::sqrt(diffx * diffx + diffy * diffy);

        if (length < .01) continue;

        diffx /= length;
        diffy /= length;

        const float spring_force =
            -1 * config.dynamic.blob_coeff * (length - config.dynamic.blob_len);

        diffx *= spring_force;
        diffy *= spring_force;

        center_p.x = diffx;
        center_p.y = diffy;
    }

    center_a.x /= points.size();
    center_a.y /= points.size();

    center_p.x = center_a.x + center_p.x;
    center_p.y = center_a.y + center_p.y;

    cv::circle(display,
               cv::Point(center_p.x * display.cols, center_p.y * display.rows),
               5, cv::Scalar(0, 0, 255), -1);
    cv::circle(display,
               cv::Point(config.dynamic.blob_x * display.cols,
                         ((float)config.dynamic.blob_y) * display.rows),
               5, cv::Scalar(0, 255, 0), -1);
    cv::circle(debug_display,
               cv::Point(center_p.x * display.cols, center_p.y * display.rows),
               5, cv::Scalar(0, 0, 255), -1);
    cv::circle(debug_display,
               cv::Point(config.dynamic.blob_x * display.cols,
                         ((float)config.dynamic.blob_y) * display.rows),
               5, cv::Scalar(0, 255, 0), -1);

    debug_img_pub_color(debug_pub_blob, display);
    debug_img_pub_bw(debug_pub_dilated, dilated);

    return center_p.x - config.dynamic.blob_x;
}

bool Blob::hasSub(){
    return twist_pub.getNumSubscribers();
}

bool Blob::isEnabled(){
    return enabled;
}

void Blob::startup(){
    image_sub = it.subscribe(camera_topic, 1, &Blob::dashcamCB, this);
    enabled = false;
}

void Blob::shutdown(){
    image_sub = image_transport::Subscriber();
    enabled =  false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "actor_ltu_route_blob");
    Blob blob;

    ros::Rate r(10); 

    while (ros::ok()){
        if (blob.hasSub()){
            if (!blob.isEnabled()){
                blob.startup();
            }
        } else {
            if (blob.isEnabled()){
                blob.shutdown();
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}
