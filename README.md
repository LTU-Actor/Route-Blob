# Route-Blob

```xml
<launch>
  <node pkg="ltu_actor_route_blob" type="blob" name="blob">
    <param name="input" type="string" value="/camera/image_raw" />
    <param name="enable_drive" type="bool" value="true" />
    <param name="enable_forward" type="bool" value="true" />
    <param name="drive_speed" type="double" value="1.5" />
    <param name="edge_method" type="int" value="0" />
    <param name="canny_lower_thresh" type="int" value="700" />
    <param name="canny_upper_thresh" type="int" value="2000" />
    <param name="canny_aperture_size" type="int" value="2" />
    <param name="adap_use_gauss" type="bool" value="false" />
    <param name="adap_block_size" type="int" value="1" />
    <param name="adap_c" type="double" value="0" />
    <param name="lapla_ksize" type="int" value="1" />
    <param name="sobel_xorder" type="int" value="1" />
    <param name="sobel_yorder" type="int" value="1" />
    <param name="sobel_ksize" type="int" value="1" />
    <param name="enhance_blur" type="int" value="4" />
    <param name="blob_y" type="double" value="0.95" />
    <param name="blob_x" type="double" value="0.5" />
    <param name="blob_coeff" type="double" value="0.1" />
    <param name="blob_len" type="double" value="0.2" />
    <param name="blob_num_points" type="int" value="50" />
    <param name="blob_median_blur_size" type="int" value="13" />
    <param name="blob_dilation_size" type="int" value="5" />
    <param name="blob_mult" type="double" value="3.0" />
    <param name="blob_max_p_y" type="double" value="0.7" />
    <param name="lines_enable" type="bool" value="false" />
    <param name="lines_thresh" type="int" value="40" />
    <param name="lines_rho" type="int" value="1" />
    <param name="lines_min_len" type="int" value="30" />
    <param name="lines_max_gap" type="int" value="10" />
    <param name="lines_top" type="double" value="0.2" />
    <param name="lines_min_slope" type="double" value="0.2" />
    <param name="show_edge_detect" type="bool" value="false" />
    <param name="show_result" type="bool" value="false" />
    <param name="show_blob" type="bool" value="false" />
    <param name="show_lines" type="bool" value="false" />
  </node>
</launch>
```
