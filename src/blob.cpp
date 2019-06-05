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
    bool enabled = false;

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
    ltu_actor_route_blob::BlobConfig config;

    void  find_edges(cv::Mat &in, cv::Mat &out);
    float blob_adjust(const cv::Mat &edges, cv::Mat &debug_display);
};

Blob::Blob()
  : nh{"~"},
    it(nh)
{
    if (!nh.getParam("input", camera_topic))
    {
        ROS_ERROR_STREAM("No camera topic passed to " + camera_topic);
        throw std::invalid_argument("Bad camera topic");
    }

    //image_sub = it.subscribe(camera_topic, 1, &Blob::dashcamCB, this);

    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd", 1);

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


    if (nh.hasParam("enable_drive")) { nh.getParam("enable_drive", config.enable_drive); }
    if (nh.hasParam("enable_forward")) { nh.getParam("enable_forward", config.enable_forward); }
    if (nh.hasParam("drive_speed")) { nh.getParam("drive_speed", config.drive_speed); }
    if (nh.hasParam("edge_method")) { nh.getParam("edge_method", config.edge_method); }
    if (nh.hasParam("canny_lower_thresh")) { nh.getParam("canny_lower_thresh", config.canny_lower_thresh); }
    if (nh.hasParam("canny_upper_thresh")) { nh.getParam("canny_upper_thresh", config.canny_upper_thresh); }
    if (nh.hasParam("canny_aperture_size")) { nh.getParam("canny_aperture_size", config.canny_aperture_size); }
    if (nh.hasParam("adap_use_gauss")) { nh.getParam("adap_use_gauss", config.adap_use_gauss); }
    if (nh.hasParam("adap_block_size")) { nh.getParam("adap_block_size", config.adap_block_size); }
    if (nh.hasParam("adap_c")) { nh.getParam("adap_c", config.adap_c); }
    if (nh.hasParam("lapla_ksize")) { nh.getParam("lapla_ksize", config.lapla_ksize); }
    if (nh.hasParam("sobel_xorder")) { nh.getParam("sobel_xorder", config.sobel_xorder); }
    if (nh.hasParam("sobel_yorder")) { nh.getParam("sobel_yorder", config.sobel_yorder); }
    if (nh.hasParam("sobel_ksize")) { nh.getParam("sobel_ksize", config.sobel_ksize); }
    if (nh.hasParam("enhance_blur")) { nh.getParam("enhance_blur", config.enhance_blur); }
    if (nh.hasParam("blob_y")) { nh.getParam("blob_y", config.blob_y); }
    if (nh.hasParam("blob_x")) { nh.getParam("blob_x", config.blob_x); }
    if (nh.hasParam("blob_coeff")) { nh.getParam("blob_coeff", config.blob_coeff); }
    if (nh.hasParam("blob_len")) { nh.getParam("blob_len", config.blob_len); }
    if (nh.hasParam("blob_num_points")) { nh.getParam("blob_num_points", config.blob_num_points); }
    if (nh.hasParam("blob_median_blur_size")) { nh.getParam("blob_median_blur_size", config.blob_median_blur_size); }
    if (nh.hasParam("blob_dilation_size")) { nh.getParam("blob_dilation_size", config.blob_dilation_size); }
    if (nh.hasParam("blob_mult")) { nh.getParam("blob_mult", config.blob_mult); }
    if (nh.hasParam("blob_max_p_y")) { nh.getParam("blob_max_p_y", config.blob_max_p_y); }
    if (nh.hasParam("lines_enable")) { nh.getParam("lines_enable", config.lines_enable); }
    if (nh.hasParam("lines_thresh")) { nh.getParam("lines_thresh", config.lines_thresh); }
    if (nh.hasParam("lines_rho")) { nh.getParam("lines_rho", config.lines_rho); }
    if (nh.hasParam("lines_min_len")) { nh.getParam("lines_min_len", config.lines_min_len); }
    if (nh.hasParam("lines_max_gap")) { nh.getParam("lines_max_gap", config.lines_max_gap); }
    if (nh.hasParam("lines_top")) { nh.getParam("lines_top", config.lines_top); }
    if (nh.hasParam("lines_min_slope")) { nh.getParam("lines_min_slope", config.lines_min_slope); }
    if (nh.hasParam("show_edge_detect")) { nh.getParam("show_edge_detect", config.show_edge_detect); }
    if (nh.hasParam("show_result")) { nh.getParam("show_result", config.show_result); }
    if (nh.hasParam("show_blob")) { nh.getParam("show_blob", config.show_blob); }
    if (nh.hasParam("show_lines")) { nh.getParam("show_lines", config.show_lines); }
    dyn_server.updateConfig(config);
}

void Blob::dynConfigCB(ltu_actor_route_blob::BlobConfig &newconfig, uint32_t level)
{
    // Mutex not needed. individual values will be atomic by x86 architecture.
    config = newconfig;
}

void Blob::dashcamCB(const sensor_msgs::ImageConstPtr &msg)
{
   // ROS_ERROR_STREAM("GOT IMAGE");
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


    cv::Mat              &input = cv_ptr->image;
    cv::Mat              edges;
    cv::Mat              hsv;
    cv::Mat              display;
    std::vector<cv::Mat> channels(3);
    
   // ROS_ERROR_STREAM("CV_BGR2HSV");

    cv::cvtColor(input, hsv, CV_BGR2HSV);
    cv::split(hsv, channels);
    // Color
    //channels[2] -= channels[1];
    cv::medianBlur(channels[2], channels[2],
                    config.enhance_blur * 2 + 1);
    cv::merge(channels, display);

    //ROS_ERROR_STREAM("CV_HSV2BGR");

    cv::cvtColor(display, display, CV_HSV2BGR);

    if (config.edge_method == 0)
        find_edges(display, edges);
    else if (config.edge_method == 1)
        find_edges(channels[2], edges);
    else if (config.edge_method == 2)
    {
        cv::adaptiveThreshold(
            channels[2], edges, 255,
            config.adap_use_gauss ? cv::ADAPTIVE_THRESH_MEAN_C
                                            : cv::ADAPTIVE_THRESH_MEAN_C,
            cv::THRESH_BINARY, config.adap_block_size * 2 + 1,
            config.adap_c);
    }
    else
    {
        cv::Laplacian(channels[2], edges, -1,
                        config.lapla_ksize * 2 + 1);
        cv::Sobel(edges, edges, -1, config.sobel_xorder,
                    config.sobel_yorder,
                    config.sobel_ksize * 2 + 1);
    }

    float turn;
    if (config.lines_enable)
    {
        cv::Mat lines_mat = cv::Mat::zeros(edges.size(), edges.type());
        std::vector<cv::Vec4i> lines;
        cv::Rect               rect =
            cv::Rect(0, config.lines_top * edges.rows, edges.cols,
                        edges.rows - config.lines_top * edges.rows);

        cv::HoughLinesP(edges(rect), lines, config.lines_rho,
                        0.01745329251, config.lines_thresh,
                        config.lines_min_len,
                        config.lines_max_gap);

        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];

            float diffx = l[0] - l[2];
            float diffy = l[1] - l[3];

            float slope = diffy / diffx;

            if (std::abs(slope) < config.lines_min_slope) continue;

            diffx *= 5;
            diffy *= 5;

            l[0] -= diffx;
            l[1] -= diffy;
            l[2] += diffx;
            l[3] += diffy;

            cv::line(
                lines_mat,
                cv::Point(l[0],
                            l[1] + config.lines_top * edges.rows),
                cv::Point(l[2],
                            l[3] + config.lines_top * edges.rows),
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
    twist.linear.x  = config.drive_speed;
    twist.angular.z = -config.blob_mult * turn;

    if (!config.enable_drive)
    {
        twist.linear.x  = 0;
        twist.angular.z = 0;
    }
    else if (!config.enable_forward)
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
    cv::Canny(in, out, config.canny_lower_thresh,
              config.canny_upper_thresh,
              config.canny_aperture_size * 2 + 1);
}

float Blob::blob_adjust(const cv::Mat &edges, cv::Mat &debug_display)
{
    cv::Mat dilated;
    cv::Mat display = cv::Mat::zeros(edges.size(), CV_8UC3);

    const int dilation_size  = config.blob_dilation_size;
    cv::Mat   dilate_element = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(dilation_size, dilation_size));

    cv::dilate(edges, dilated, dilate_element);

    if (config.blob_median_blur_size > 0)
        cv::medianBlur(dilated, dilated, config.blob_median_blur_size * 2 + 1);

    struct Point {
        float x;
        float y;
    };

    std::vector<Point> points;
    points.reserve(config.blob_num_points + 1);

    for (float theta = 0; theta <= M_PI;
         theta += M_PI / config.blob_num_points)
    {
        Point p;
        p.x = config.blob_x;
        p.y = config.blob_y;

        float diffx = std::cos(theta) * .01;
        float diffy = -1 * std::sin(theta) * .01;

        while (dilated.at<uint8_t>(p.y * dilated.rows, p.x * dilated.cols)
               < config.blob_num_points)
        {
            p.x += diffx;
            p.y += diffy;

            float top_y = 1 - config.blob_max_p_y;
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

    Point center_p = {(float)config.blob_x, (float)config.blob_y};
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
            -1 * config.blob_coeff * (length - config.blob_len);

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
               cv::Point(config.blob_x * display.cols,
                         ((float)config.blob_y) * display.rows),
               5, cv::Scalar(0, 255, 0), -1);
    cv::circle(debug_display,
               cv::Point(center_p.x * display.cols, center_p.y * display.rows),
               5, cv::Scalar(0, 0, 255), -1);
    cv::circle(debug_display,
               cv::Point(config.blob_x * display.cols,
                         ((float)config.blob_y) * display.rows),
               5, cv::Scalar(0, 255, 0), -1);

    debug_img_pub_color(debug_pub_blob, display);
    debug_img_pub_bw(debug_pub_dilated, dilated);

    return center_p.x - config.blob_x;
}

bool Blob::hasSub(){
    return (twist_pub.getNumSubscribers() || debug_pub_blob.getNumSubscribers() ||
            debug_pub_lines.getNumSubscribers() || debug_pub_edges.getNumSubscribers() || debug_pub_result.getNumSubscribers());

}

bool Blob::isEnabled(){
    return enabled;
}

void Blob::startup(){
    image_sub = it.subscribe(camera_topic, 1, &Blob::dashcamCB, this);
    enabled = true;
}

void Blob::shutdown(){
    image_sub = image_transport::Subscriber();
    enabled =  false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "actor_ltu_route_blob");
    Blob blob;

    ros::Rate r(30); 

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
