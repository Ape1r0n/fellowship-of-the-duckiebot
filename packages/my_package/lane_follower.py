#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from collections import deque
import time

# Tunable parameters - Adjusted for smoother turning
BASE_SPEED = 0.18          # Reduced base speed for better control
CURVE_SPEED = 0.16         # Further reduced speed for curves
P_GAIN = 0.35              # Reduced proportional gain for smoother response
D_GAIN = 0.25              # Increased derivative gain for better curve handling
MAX_STEER = 0.2            # Reduced maximum steering adjustment
SMOOTHING_STRAIGHT = 5     # Increased smoothing for straight roads
SMOOTHING_CURVE = 3        # Moderate smoothing for curves
TURN_SLOWDOWN_FACTOR = 0.7 # Factor to slow down during turns

class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        super(CameraReaderNode, self).__init__(
            node_name=node_name, node_type=NodeType.VISUALIZATION)

        # Initialize parameters
        self.base_speed = BASE_SPEED
        self.curve_speed = CURVE_SPEED
        self.p_gain = P_GAIN
        self.d_gain = D_GAIN
        self.max_steer = MAX_STEER
        
        # Lane following state tracking
        self.start_time = time.time()
        
        # Error tracking with more smoothing
        self.prev_error = 0
        self.error_history = deque(maxlen=5)  # Additional error smoothing
        self.left_motor_history = deque(maxlen=SMOOTHING_STRAIGHT)
        self.right_motor_history = deque(maxlen=SMOOTHING_STRAIGHT)
        
        # Line tracking for better curve handling
        self.yellow_centroid_history = deque(maxlen=5)
        self.white_centroid_history = deque(maxlen=5)
        self.lane_width_history = deque(maxlen=10)
        
        # Setup ROS nodes
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"
        self._window = "camera-reader"
        self._debug_window = "debug-masks"
        self.bridge = CvBridge()
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow(self._debug_window, cv2.WINDOW_AUTOSIZE)

        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self.left_motor = rospy.Publisher("left_motor", Float64, queue_size=1)
        self.right_motor = rospy.Publisher("right_motor", Float64, queue_size=1)

        self.shutting_down = False
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.shutting_down = True
        self.left_motor.publish(0)
        self.right_motor.publish(0)
        cv2.destroyAllWindows()
        
    def smooth_motor_value(self, value, history_buffer):
        """Apply smoothing to motor values with weighted average"""
        history_buffer.append(value)
        # Use weighted average (more weight to recent values)
        weights = np.linspace(1, 2, len(history_buffer))
        return np.average(history_buffer, weights=weights)

    def detect_line_curvature(self, mask):
        """Detect if a line is curving and in which direction"""
        if mask is None or np.count_nonzero(mask) < 50:
            return 0, False  # No curvature info available
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0, False
        
        # Get the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Fit a line to the contour points
        if len(largest_contour) < 10:
            return 0, False
        
        # Extract x,y coordinates
        points = largest_contour.reshape(-1, 2)
        
        # Sort points by y-coordinate (top to bottom)
        points = points[points[:, 1].argsort()]
        
        # Split into top and bottom sections
        mid_idx = len(points) // 2
        top_points = points[:mid_idx]
        bottom_points = points[mid_idx:]
        
        if len(top_points) < 3 or len(bottom_points) < 3:
            return 0, False
        
        # Calculate average x-position for top and bottom sections
        top_x_avg = np.mean(top_points[:, 0])
        bottom_x_avg = np.mean(bottom_points[:, 0])
        
        # Calculate curvature (positive = curving right, negative = curving left)
        curvature = top_x_avg - bottom_x_avg
        is_curving = abs(curvature) > 20  # Threshold for detecting significant curvature
        
        return curvature, is_curving

    def estimate_lane_width(self, yellow_centroid, white_centroid):
        """Estimate current lane width and maintain running average"""
        if yellow_centroid is not None and white_centroid is not None:
            current_width = abs(yellow_centroid[0] - white_centroid[0])
            self.lane_width_history.append(current_width)
            return np.mean(self.lane_width_history)
        elif len(self.lane_width_history) > 0:
            return np.mean(self.lane_width_history)
        else:
            return 200  # Default assumption

    def calculate_line_following_error(self, yellow_centroid, white_centroid, w, yellow_mask=None, white_mask=None):
        """Calculate error to keep robot in center between lines, with improved single-line handling"""
        ideal_center = w / 2
        
        # Update centroid history for trend analysis
        self.yellow_centroid_history.append(yellow_centroid)
        self.white_centroid_history.append(white_centroid)
        
        # Detect line curvature
        yellow_curvature, yellow_curving = self.detect_line_curvature(yellow_mask)
        white_curvature, white_curving = self.detect_line_curvature(white_mask)
        
        # Estimate current lane width
        estimated_lane_width = self.estimate_lane_width(yellow_centroid, white_centroid)
        
        if yellow_centroid is not None and white_centroid is not None:
            # Both lines visible - stay in the middle between them
            lane_center = (yellow_centroid[0] + white_centroid[0]) / 2
            error = ideal_center - lane_center
            line_spacing = abs(yellow_centroid[0] - white_centroid[0])
            return error, line_spacing, "both_lines", lane_center
            
        elif yellow_centroid is not None:
            # Only yellow line visible - improved handling for curves
            if yellow_curving and yellow_curvature < -15:  # Yellow line curving left significantly
                # When yellow line curves left, stay closer to it to avoid crossing
                # Reduce the offset distance based on curvature intensity
                curvature_factor = max(0.3, 1 - abs(yellow_curvature) / 100)
                safe_distance = estimated_lane_width * 0.3 * curvature_factor  # Closer following
                lane_center = yellow_centroid[0] + safe_distance
            else:
                # Normal yellow line following - position to the right
                lane_center = yellow_centroid[0] + (estimated_lane_width * 0.4)
            
            error = ideal_center - lane_center
            return error, None, f"yellow_only_curve_{yellow_curvature:.1f}", lane_center
            
        elif white_centroid is not None:
            # Only white line visible - improved handling for curves
            if white_curving and white_curvature > 15:  # White line curving right significantly
                # When white line curves right, stay closer to it
                curvature_factor = max(0.3, 1 - abs(white_curvature) / 100)
                safe_distance = estimated_lane_width * 0.3 * curvature_factor
                lane_center = white_centroid[0] - safe_distance
            else:
                # Normal white line following - position to the left
                lane_center = white_centroid[0] - (estimated_lane_width * 0.4)
            
            error = ideal_center - lane_center
            return error, None, f"white_only_curve_{white_curvature:.1f}", lane_center
            
        else:
            # No lines visible - use previous error with decay
            error = self.prev_error * 0.8
            return error, None, "no_lines", None

    def callback(self, msg):
        if self.shutting_down:
            return

        # Process image with less aggressive filtering
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.image = cv2.GaussianBlur(self.image, (5, 5), 0)  # Lighter blur
        
        # Create a copy for visualization
        vis_image = self.image.copy()
        
        # Define regions of interest
        h, w = self.image.shape[:2]
        roi_top = int(h*0.6)  # ROI starts at 60% down from top
        roi_bottom = self.image[roi_top:, :]  # Bottom 40% of image
        
        # DEBUG: Draw ROI rectangle on visualization
        cv2.rectangle(vis_image, (0, roi_top), (w-1, h-1), (255, 0, 0), 2)
        cv2.putText(vis_image, "ROI", (10, roi_top-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Color space conversions
        luv = cv2.cvtColor(roi_bottom, cv2.COLOR_BGR2LUV)
        hls = cv2.cvtColor(roi_bottom, cv2.COLOR_BGR2HLS)

        # Yellow line detection (left boundary)
        lb_yellow = np.array([10, 85, 160])
        ub_yellow = np.array([255, 255, 255])
        mask_yellow = cv2.inRange(luv, lb_yellow, ub_yellow)
        
        # White line detection (right boundary)
        lb_white = np.array([0, 180, 0])  # Higher lightness threshold
        ub_white = np.array([172, 255, 255])
        mask_white = cv2.inRange(hls, lb_white, ub_white)
        
        # Apply morphological operations
        kernel = np.ones((3, 3), np.uint8)  # Smaller kernel
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
        
        # Find line centroids
        yellow_centroid = self.find_line_centroid(mask_yellow)
        white_centroid = self.find_line_centroid(mask_white)
        
        # Calculate error based on lane-following logic with curvature awareness
        error, line_spacing, following_mode, target_position = self.calculate_line_following_error(
            yellow_centroid, white_centroid, w, mask_yellow, mask_white)
        
        # DEBUG: Create debug visualization showing all masks and detected pixels
        debug_image = np.zeros((h, w, 3), dtype=np.uint8)
        
        # Show ROI area
        debug_image[roi_top:, :] = [50, 50, 50]  # Dark gray for ROI
        
        # Count pixels for debugging
        yellow_pixel_count = np.count_nonzero(mask_yellow)
        white_pixel_count = np.count_nonzero(mask_white)
        total_line_pixels = yellow_pixel_count + white_pixel_count
        
        # Overlay yellow mask pixels in yellow
        yellow_pixels = np.where(mask_yellow > 0)
        if len(yellow_pixels[0]) > 0:
            debug_image[yellow_pixels[0] + roi_top, yellow_pixels[1]] = [0, 255, 255]  # Yellow
            
        # Overlay white mask pixels in white
        white_pixels = np.where(mask_white > 0)
        if len(white_pixels[0]) > 0:
            debug_image[white_pixels[0] + roi_top, white_pixels[1]] = [255, 255, 255]  # White
        
        # DEBUG: Mark centroids and target positions on both images
        if yellow_centroid is not None:
            centroid_y_global = yellow_centroid[1] + roi_top
            cv2.circle(vis_image, (yellow_centroid[0], centroid_y_global), 8, (0, 255, 255), 2)
            cv2.circle(debug_image, (yellow_centroid[0], centroid_y_global), 8, (0, 255, 255), 2)
            cv2.putText(debug_image, "Y", (yellow_centroid[0]-10, centroid_y_global-15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        if white_centroid is not None:
            centroid_y_global = white_centroid[1] + roi_top
            cv2.circle(vis_image, (white_centroid[0], centroid_y_global), 8, (255, 255, 255), 2)
            cv2.circle(debug_image, (white_centroid[0], centroid_y_global), 8, (255, 255, 255), 2)
            cv2.putText(debug_image, "W", (white_centroid[0]-10, centroid_y_global-15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Draw target position and ideal center
        if target_position is not None:
            cv2.line(vis_image, (int(target_position), roi_top), 
                    (int(target_position), h), (0, 255, 0), 3)
            cv2.line(debug_image, (int(target_position), roi_top), 
                    (int(target_position), h), (0, 255, 0), 3)
        
        ideal_center = w / 2
        cv2.line(vis_image, (int(ideal_center), roi_top), 
                (int(ideal_center), h), (255, 0, 0), 2)
        cv2.line(debug_image, (int(ideal_center), roi_top), 
                (int(ideal_center), h), (255, 0, 0), 2)
        
        # Normalize error to range [-1, 1]
        error = np.clip(error / (w/2), -1, 1)
        self.error_history.append(error)
        smoothed_error = np.mean(self.error_history)  # Additional error smoothing
        
        # Calculate derivative of error for D-term
        error_diff = smoothed_error - self.prev_error
        self.prev_error = smoothed_error
        
        # Enhanced PD control with curvature-aware steering
        steering = self.p_gain * smoothed_error + self.d_gain * error_diff
        
        # Apply additional steering adjustment for single-line curve following
        if "yellow_only" in following_mode and yellow_centroid is not None:
            # Detect if yellow line is curving left (negative curvature)
            yellow_curvature, _ = self.detect_line_curvature(mask_yellow)
            if yellow_curvature < -20:  # Significant left curve
                # Add extra left steering to follow the curve more aggressively
                curve_adjustment = -0.1 * (abs(yellow_curvature) / 50)
                steering += curve_adjustment
                
        elif "white_only" in following_mode and white_centroid is not None:
            # Similar adjustment for white line right curves
            white_curvature, _ = self.detect_line_curvature(mask_white)
            if white_curvature > 20:  # Significant right curve
                curve_adjustment = 0.1 * (abs(white_curvature) / 50)
                steering += curve_adjustment
        
        steering = np.clip(steering, -self.max_steer, self.max_steer)
        
        # Detect curves based on line spacing and error
        is_curve = False
        if line_spacing is not None and line_spacing < 180:  # Adjusted threshold
            is_curve = True
        elif abs(smoothed_error) > 0.3:  # High error also indicates curve
            is_curve = True
        
        # Adjust speeds based on curve detection and steering amount
        if is_curve or abs(steering) > 0.1:
            current_speed = self.curve_speed
            # Slow down more when steering harder
            speed_factor = 1 - (abs(steering) * TURN_SLOWDOWN_FACTOR)
            current_speed *= max(0.4, speed_factor)  # Minimum 40% speed for tight turns
        else:
            current_speed = self.base_speed
        
        # Calculate motor values with smoother transitions
        left_motor = current_speed - steering
        right_motor = current_speed + steering
        
        # Enhanced recovery behavior if no lines detected
        if total_line_pixels < 300:  # Very few line pixels detected
            # More conservative recovery - slow down and gentle search pattern
            recovery_speed = current_speed * 0.3
            if len(self.yellow_centroid_history) > 0 and any(c is not None for c in self.yellow_centroid_history):
                # Had yellow line recently - search left
                left_motor = recovery_speed - 0.05
                right_motor = recovery_speed + 0.05
            elif len(self.white_centroid_history) > 0 and any(c is not None for c in self.white_centroid_history):
                # Had white line recently - search right
                left_motor = recovery_speed + 0.05
                right_motor = recovery_speed - 0.05
            else:
                # Default gentle left search
                left_motor = recovery_speed - 0.02
                right_motor = recovery_speed + 0.02
        
        # Apply smoothing based on curve detection
        smoothing_amount = SMOOTHING_CURVE if is_curve else SMOOTHING_STRAIGHT
        self.left_motor_history = deque(self.left_motor_history, maxlen=smoothing_amount)
        self.right_motor_history = deque(self.right_motor_history, maxlen=smoothing_amount)
        
        # Apply weighted smoothing
        left_motor = self.smooth_motor_value(left_motor, self.left_motor_history)
        right_motor = self.smooth_motor_value(right_motor, self.right_motor_history)
        
        # Safety bounds
        left_motor = np.clip(left_motor, -1.0, 1.0)
        right_motor = np.clip(right_motor, -1.0, 1.0)
        
        # Publish motor commands
        if not self.shutting_down:
            self.left_motor.publish(left_motor)
            self.right_motor.publish(right_motor)

        # Enhanced display visualization with debug info
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Show current following mode prominently
        if "both_lines" in following_mode:
            mode_text = "LANE CENTER (Both Lines)"
            mode_color = (0, 255, 0)
        elif "yellow_only" in following_mode:
            mode_text = f"YELLOW LINE ONLY ({following_mode.split('_')[-1]})"
            mode_color = (0, 255, 255)
        elif "white_only" in following_mode:
            mode_text = f"WHITE LINE ONLY ({following_mode.split('_')[-1]})"
            mode_color = (255, 255, 255)
        else:
            mode_text = "NO LINES DETECTED"
            mode_color = (0, 0, 255)
            
        cv2.putText(vis_image, mode_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
        
        cv2.putText(vis_image, f"Time: {elapsed_time:.1f}s", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Steering: {steering:.2f}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Speed: {current_speed:.2f}", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Error: {smoothed_error:.2f}", (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(vis_image, f"Lane Width: {self.estimate_lane_width(yellow_centroid, white_centroid):.0f}px", (10, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Debug window information
        cv2.putText(debug_image, mode_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)
        cv2.putText(debug_image, f"Yellow pixels: {yellow_pixel_count}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cv2.putText(debug_image, f"White pixels: {white_pixel_count}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(debug_image, f"Total line pixels: {total_line_pixels}", (10, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        cv2.putText(debug_image, f"Following mode: {following_mode}", (10, 150), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        if line_spacing is not None:
            cv2.putText(debug_image, f"Line spacing: {line_spacing:.1f}px", (10, 180), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Add curvature information
        if yellow_centroid is not None:
            yellow_curvature, _ = self.detect_line_curvature(mask_yellow)
            cv2.putText(debug_image, f"Yellow curve: {yellow_curvature:.1f}", (10, 210), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        if white_centroid is not None:
            white_curvature, _ = self.detect_line_curvature(mask_white)
            cv2.putText(debug_image, f"White curve: {white_curvature:.1f}", (10, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Add legend to debug window
        legend_y = h - 140
        cv2.rectangle(debug_image, (10, legend_y), (400, h-10), (0, 0, 0), -1)
        cv2.putText(debug_image, "Legend:", (15, legend_y + 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(debug_image, "Yellow: Left lane boundary", (15, legend_y + 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        cv2.putText(debug_image, "White: Right lane boundary", (15, legend_y + 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(debug_image, "Green: Target position (adaptive)", (15, legend_y + 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(debug_image, "Blue: Robot center reference", (15, legend_y + 75), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
        cv2.putText(debug_image, "Curve-aware following: Closer to curving lines", (15, legend_y + 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        cv2.putText(debug_image, "Negative curvature = Left turn, Positive = Right turn", (15, legend_y + 105), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        cv2.imshow(self._window, vis_image)
        cv2.imshow(self._debug_window, debug_image)
        cv2.waitKey(1)

    def find_line_centroid(self, mask):
        """Find the centroid of the largest contour in the mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Calculate moments and centroid
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
            
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)

if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    rospy.spin()