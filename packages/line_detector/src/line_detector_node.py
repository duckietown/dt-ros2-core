#!/usr/bin/env python3
import json
from typing import List, Dict
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Segment as SegmentMsg, SegmentList, AntiInstagramThresholds
from dt_computer_vision.line_detection import LineDetector, ColorRange, Detections
from dt_computer_vision.line_detection.rendering import draw_segments, draw_maps
from dt_computer_vision.anti_instagram import AntiInstagram


class LineDetectorNode(Node):
    """
    The ``LineDetectorNode`` detects white, yellow, and red line segments in images for lane localization (ROS 2).

    Upon receiving an image, this node reduces its resolution, crops the top part so only the
    road-containing portion remains, extracts white/red/yellow segments, and publishes them.
    The main functionality is implemented by :py:class:`line_detector.LineDetector`.

    The performance of this node can be very sensitive to its configuration parameters. Therefore, it also provides
    several debug topics to fine-tune these parameters. These configuration parameters can be changed dynamically via
    ROS 2 parameters (e.g., using ``ros2 param set``) and parameter files.

    Args:
        node_name (:obj:`str`): Unique, descriptive node name.

    Configuration (ROS 2 parameters):
        line_detector_parameters.* (:obj:`int|float|list`): Parameters for the detector; see :py:class:`line_detector.LineDetector`.
        colors.* (:obj:`list[int]`): Color ranges to detect. Keys must match Segment msg color names.
        img_size (:obj:`list[int]`): Downsized resolution [H, W]; lower is faster but less accurate. Default: [120, 160].
        top_cutoff (:obj:`int`): Rows to remove from the top after resizing. Default: 40.
        traffic_mode (:obj:`str`): Either "RHT" or "LHT" to mirror images for left-hand traffic. Default: "RHT".

    Subscriptions:
        image/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Camera images.
        thresholds (:obj:`duckietown_msgs.msg.AntiInstagramThresholds`): Thresholds for color correction.

    Publications:
        segment_list (:obj:`duckietown_msgs.msg.SegmentList`): List of detected segments.
        debug/segments/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Segments drawn on the input image.
        debug/edges/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Canny edges drawn on the input image.
        debug/maps/compressed (:obj:`sensor_msgs.msg.CompressedImage`): Regions per color range drawn on the input image.

    """

    def __init__(self, node_name: str = "line_detector_node"):
        super().__init__(node_name, automatically_declare_parameters_from_overrides=True)

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Get parameters (already declared automatically from overrides)
        # Set defaults if parameters don't exist
        try:
            self._img_size = self.get_parameter("img_size").get_parameter_value().integer_array_value
            if not self._img_size:
                self._img_size = [120, 160]
            else:
                self._img_size = list(self._img_size)
        except:
            self.declare_parameter("img_size", [120, 160])
            self._img_size = [120, 160]
            
        try:
            self._top_cutoff = int(self.get_parameter("top_cutoff").value)
        except:
            self.declare_parameter("top_cutoff", 40)
            self._top_cutoff = 40
            
        try:
            self._traffic_mode = str(self.get_parameter("traffic_mode").value)
        except:
            self.declare_parameter("traffic_mode", "RHT")
            self._traffic_mode = "RHT"

        self.bridge = CvBridge()

        # AntiInstagram thresholds
        self.ai_thresholds_received = False
        self.anti_instagram_thresholds = dict()
        self.ai = AntiInstagram()

        # Cache for debug/ranges image color maps
        self.colormaps = dict()

        # Read detector parameters under prefix 'line_detector_parameters'
        ldp = self.get_parameters_by_prefix("line_detector_parameters")
        ldp_clean = {k: v.value for k, v in ldp.items()}
        self.detector = LineDetector(**ldp_clean)

        # Default color ranges from config file
        self.default_colors = {
            "RED": {
                "low_1": [0, 140, 100],
                "high_1": [15, 255, 255],
                "low_2": [165, 140, 100],
                "high_2": [180, 255, 255]
            },
            "WHITE": {
                "low": [0, 0, 150],
                "high": [180, 100, 255]
            },
            "YELLOW": {
                "low": [25, 140, 100],
                "high": [45, 255, 255]
            }
        }

        # Initialize color ranges from parameters under prefix 'colors'
        self.color_ranges: Dict[str, ColorRange] = {}
        self.on_colors_range_change()

        # Register parameter update callback for dynamic updates
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        # Publishers
        self.pub_lines = self.create_publisher(SegmentList, "segment_list", 10)
        self.pub_d_segments = self.create_publisher(CompressedImage, "debug/segments/compressed", sensor_qos)
        self.pub_d_edges = self.create_publisher(CompressedImage, "debug/edges/compressed", sensor_qos)
        self.pub_d_maps = self.create_publisher(CompressedImage, "debug/maps/compressed", sensor_qos)

        # Subscribers
        self.sub_image = self.create_subscription(
            CompressedImage, "image/compressed", self.image_cb, sensor_qos
        )
        self.sub_thresholds = self.create_subscription(
            AntiInstagramThresholds, "thresholds", self.thresholds_cb, 10
        )

        # Check if CUDA is available
        if cv2.cuda.getCudaEnabledDeviceCount() > 0:
            self.get_logger().info("Using CUDA GPU for line detection.")
            self.cuda_enabled = True
        else:
            self.get_logger().info("Using the CPU for line detection.")
            self.cuda_enabled = False

    def on_colors_range_change(self):
        # Read parameters under 'colors.' prefix and rebuild the dict structure
        params = self.get_parameters_by_prefix("colors")
        # Build color dict as expected by ColorRange.fromDict
        colors: Dict[str, Dict] = {}
        for full_key, param in params.items():
            # full_key examples: 'RED.low_1', 'WHITE.low', 'YELLOW.high'
            parts = full_key.split(".")
            if len(parts) != 2:
                continue
            color, key = parts
            if color not in colors:
                colors[color] = {}
            colors[color][key] = param.value

        # If no color parameters found, use defaults
        if not colors:
            colors = self.default_colors
            self.get_logger().info("No color parameters found, using default values")

        self.color_ranges = {
            color: ColorRange.fromDict(d) for color, d in colors.items()
        }
        try:
            self.get_logger().info(f"Color range changed to {json.dumps(colors)}")
        except Exception:
            self.get_logger().info("Color range parameters updated.")

    def thresholds_cb(self, thresh_msg: AntiInstagramThresholds):
        self.anti_instagram_thresholds["lower"] = thresh_msg.low
        self.anti_instagram_thresholds["higher"] = thresh_msg.high
        self.ai_thresholds_received = True

    def image_cb(self, image_msg: CompressedImage):
        """
        Processes the incoming image messages.

        Performs the following steps for each incoming image:

        #. Performs color correction
        #. Resizes the image to the ``~img_size`` resolution
        #. Removes the top ``~top_cutoff`` rows in order to remove the part of the image that doesn't include the road
        #. Extracts the line segments in the image using :py:class:`line_detector.LineDetector`
        #. Converts the coordinates of detected segments to normalized ones
        #. Creates and publishes the resultant :obj:`duckietown_msgs.msg.SegmentList` message
        #. Creates and publishes debug images if there is a subscriber to the respective topics

        Args:
            image_msg (:obj:`sensor_msgs.msg.CompressedImage`): The receive image message

        """

        # Decode from compressed image with OpenCV
        try:
            obtained_image = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.get_logger().error(f"Could not decode image: {e}")
            return
        
        # Perform color correction
        if self.ai_thresholds_received:
            obtained_image = self.ai.apply(
                image = obtained_image,
                lower_threshold = self.anti_instagram_thresholds["lower"],
                higher_threshold = self.anti_instagram_thresholds["higher"]
            )

        if self.cuda_enabled:
            gpu_image = cv2.cuda_GpuMat()
            gpu_image.upload(obtained_image)
        else:
            gpu_image = obtained_image

        # Resize the gpu_image to the desired dimensions
        height_original, width_original = gpu_image.shape[0:2]
        img_size = (self._img_size[1], self._img_size[0])
        if img_size[0] != width_original or img_size[1] != height_original:
            if self.cuda_enabled:
                gpu_image = cv2.cuda.resize(gpu_image, img_size, interpolation=cv2.INTER_NEAREST)
            else:
                gpu_image = cv2.resize(gpu_image, img_size, interpolation=cv2.INTER_NEAREST)

        gpu_image = gpu_image[self._top_cutoff :, :, :]

        # mirror the gpu_image if left-hand traffic mode is set
        if self._traffic_mode == "LHT":
            gpu_image = np.fliplr(gpu_image)

        color_order = ["YELLOW", "WHITE", "RED"]
        colors_to_detect = [self.color_ranges[c] for c in color_order]
        # Extract the line segments for every color
        color_detections: List[Detections] = (
            self.detector.detect(gpu_image, colors_to_detect))

        dets: Dict[str, dict] ={}
        for i, detections in enumerate(color_detections):
            color = color_order[i]
            # pack detections in a dictionary
            dets[color] = {
                "lines": detections.lines.tolist(),
                "centers": detections.centers.tolist(),
                "normals": detections.normals.tolist(),
                "color": self.color_ranges[color].representative
            }

        # Construct a SegmentList
        segment_list = SegmentList()
        segment_list.header.stamp = image_msg.header.stamp

        # Remove the offset in coordinates coming from the removing of the top part and
        arr_cutoff = np.array([0, self._top_cutoff, 0, self._top_cutoff])
        arr_ratio = np.array(
            [
                1.0 / self._img_size[1],
                1.0 / self._img_size[0],
                1.0 / self._img_size[1],
                1.0 / self._img_size[0],
            ]
        )

        # Fill in the segment_list with all the detected segments
        for color, det in dets.items():
            # Get the ID for the color from the Segment msg definition
            # Throw and exception otherwise
            if len(det["lines"]) > 0 and len(det["normals"]) > 0:
                try:
                    color_id = getattr(SegmentMsg, color)
                    lines_normalized = (det["lines"] + arr_cutoff) * arr_ratio
                    segment_list.segments.extend(
                        self._to_segment_msg(lines_normalized, det["normals"], color_id)
                    )
                except AttributeError:
                    self.logerr(f"Color name {color} is not defined in the Segment type")

        # Publish the message
        self.pub_lines.publish(segment_list)
        
        if self.cuda_enabled:
            # Download the image from gpu memory
            image = gpu_image.download()
        else:
            # Just rename appropriately the image variable
            image = gpu_image

        # If there are any subscribers to the debug topics, generate a debug image and publish it
        if self.pub_d_segments.get_subscription_count() > 0:
            debug_img = draw_segments(image,
                                      {
                                        self.color_ranges["YELLOW"]: color_detections[0],
                                        self.color_ranges["WHITE"]: color_detections[1],
                                        self.color_ranges["RED"]: color_detections[2]
                                      }
                                    )

            # mirror the image if left-hand traffic mode is set
            if self._traffic_mode == "LHT":
                debug_img = np.fliplr(debug_img)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_segments.publish(debug_image_msg)

        if self.pub_d_edges.get_subscription_count() > 0:
            canny_edges = self.detector.find_edges(image,
                                                   self.detector.canny_thresholds[0],
                                                   self.detector.canny_thresholds[1],
                                                   self.detector.canny_aperture_size
                                                   )
            # mirror the image if left-hand traffic mode is set
            if self._traffic_mode == "LHT":
                canny_edges = np.fliplr(canny_edges)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(canny_edges)
            debug_image_msg.header = image_msg.header
            self.pub_d_edges.publish(debug_image_msg)

        if self.pub_d_maps.get_subscription_count() > 0:
            debug_img = draw_maps(image,
                                  {
                                      self.color_ranges["YELLOW"]: color_detections[0],
                                      self.color_ranges["WHITE"]: color_detections[1],
                                      self.color_ranges["RED"]: color_detections[2],
                                  }
                                  )
            # mirror the image if left-hand traffic mode is set
            if self._traffic_mode == "LHT":
                debug_img = np.fliplr(debug_img)
            debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
            debug_image_msg.header = image_msg.header
            self.pub_d_maps.publish(debug_image_msg)


    @staticmethod
    def _to_segment_msg(lines, normals, color):
        """
        Converts line detections to a list of Segment messages.

        Converts the resultant line segments and normals from the line detection to a list of Segment messages.

        Args:
            lines (:obj:`numpy array`): An ``Nx4`` array where each row represents a line.
            normals (:obj:`numpy array`): An ``Nx2`` array where each row represents the normal of a line.
            color (:obj:`str`): Color name string, should be one of the pre-defined in the Segment message definition.

        Returns:
            :obj:`list` of :obj:`duckietown_msgs.msg.Segment`: List of Segment messages

        """
        segment_msg_list = []
        for x1, y1, x2, y2, norm_x, norm_y in np.hstack((lines, normals)):
            segment = SegmentMsg()
            segment.color = color
            segment.pixels_normalized[0].x = x1
            segment.pixels_normalized[0].y = y1
            segment.pixels_normalized[1].x = x2
            segment.pixels_normalized[1].y = y2
            segment.normal.x = norm_x
            segment.normal.y = norm_y
            segment_msg_list.append(segment)
        return segment_msg_list

    def _plot_ranges_histogram(self, channels):
        """Utility method for plotting color histograms and color ranges.

        Args:
            channels (:obj:`str`): The desired two channels, should be one of ``['HS','SV','HV']``

        Returns:
            :obj:`numpy array`: The resultant plot image

        """
        channel_to_axis = {"H": 0, "S": 1, "V": 2}
        axis_to_range = {0: 180, 1: 256, 2: 256}

        # Get which is the third channel that will not be shown in this plot
        missing_channel = "HSV".replace(channels[0], "").replace(channels[1], "")

        hsv_im = self.detector.hsv
        # Get the pixels as a list (flatten the horizontal and vertical dimensions)
        hsv_im = hsv_im.reshape((-1, 3))

        channel_idx = [channel_to_axis[channels[0]], channel_to_axis[channels[1]]]

        # Get only the relevant channels
        x_bins = np.arange(0, axis_to_range[channel_idx[1]] + 1, 2)
        y_bins = np.arange(0, axis_to_range[channel_idx[0]] + 1, 2)
        h, _, _ = np.histogram2d(
            x=hsv_im[:, channel_idx[0]], y=hsv_im[:, channel_idx[1]], bins=[y_bins, x_bins]
        )
        # Log-normalized histogram
        np.log(h, out=h, where=(h != 0))
        h = (255 * h / np.max(h)).astype(np.uint8)

        # Make a color map, for the missing channel, just take the middle of the range
        if channels not in self.colormaps:
            colormap_1, colormap_0 = np.meshgrid(x_bins[:-1], y_bins[:-1])
            colormap_2 = np.ones_like(colormap_0) * (axis_to_range[channel_to_axis[missing_channel]] / 2)

            channel_to_map = {channels[0]: colormap_0, channels[1]: colormap_1, missing_channel: colormap_2}

            self.colormaps[channels] = np.stack(
                [channel_to_map["H"], channel_to_map["S"], channel_to_map["V"]], axis=-1
            ).astype(np.uint8)

            if self.cuda_enabled:
                self.colormaps[channels] = cv2.cuda.cvtColor(self.colormaps[channels], cv2.COLOR_HSV2BGR)
            else:
                self.colormaps[channels] = cv2.cvtColor(self.colormaps[channels], cv2.COLOR_HSV2BGR)

        # resulting histogram image as a blend of the two images
        if self.cuda_enabled:
            im = cv2.cuda.cvtColor(h[:, :, None], cv2.COLOR_GRAY2BGR)
        else:
            im = cv2.cvtColor(h[:, :, None], cv2.COLOR_GRAY2BGR)
            
        im = cv2.addWeighted(im, 0.5, self.colormaps[channels], 1 - 0.5, 0.0)

        # now plot the color ranges on top
        for _, color_range in list(self.color_ranges.items()):
            # convert HSV color to BGR
            c = color_range.representative
            c = np.uint8([[[c[0], c[1], c[2]]]])
            if self.cuda_enabled:
                color = cv2.cuda.cvtColor(c, cv2.COLOR_HSV2BGR).squeeze().astype(int).tolist()
            else:
                color = cv2.cvtColor(c, cv2.COLOR_HSV2BGR).squeeze().astype(int).tolist()

            for i in range(len(color_range.low)):
                cv2.rectangle(
                    im,
                    pt1=(
                        (color_range.high[i, channel_idx[1]] / 2).astype(np.uint8),
                        (color_range.high[i, channel_idx[0]] / 2).astype(np.uint8),
                    ),
                    pt2=(
                        (color_range.low[i, channel_idx[1]] / 2).astype(np.uint8),
                        (color_range.low[i, channel_idx[0]] / 2).astype(np.uint8),
                    ),
                    color=color,
                    lineType=cv2.LINE_4,
                )
        # ---
        return im


    def _on_parameters_changed(self, params):
        # Update cached parameters on change; accept all valid updates
        updated = False
        for p in params:
            if p.name.startswith("colors."):
                updated = True
            elif p.name == "img_size":
                try:
                    self._img_size = list(p.value)
                except Exception:
                    pass
            elif p.name == "top_cutoff":
                try:
                    self._top_cutoff = int(p.value)
                except Exception:
                    pass
            elif p.name == "traffic_mode":
                try:
                    self._traffic_mode = str(p.value)
                except Exception:
                    pass
        if updated:
            self.on_colors_range_change()
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
