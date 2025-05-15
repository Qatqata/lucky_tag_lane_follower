#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import apriltag
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import math

class AprilTagPublisher:
    def __init__(self):
        # Initialize the running flag first
        self.running = True

        rospy.init_node('april_tag_publisher_node')

        # Get the robot name from the environment variable
        self.robot_name = rospy.get_param('~veh', 'lucky')

        # Subscribe to the camera image topic
        self.sub = rospy.Subscriber(f'/{self.robot_name}/camera_node/image/compressed', CompressedImage, self.image_callback)
        # Publish the AprilTag detections
        self.pub = rospy.Publisher('/apriltag_detections', AprilTagDetectionArray, queue_size=10)
        # Publish the tag poses
        self.pose_pub = rospy.Publisher('/apriltag_poses', PoseStamped, queue_size=10)
        # Publish tag messages based on pose
        self.message_pub = rospy.Publisher('/apriltag_messages', String, queue_size=10)
        
        rospy.loginfo("Initialized pose publisher on topic /apriltag_poses")
        rospy.loginfo("Initialized message publisher on topic /apriltag_messages")

        # Initialize TF broadcaster and listener
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        # Camera calibration parameters (you'll need to replace these with your actual camera parameters)
        self.camera_matrix = np.array([
            [307.1974162863454, 0, 335.5197921130312],
            [0, 305.9406329104927, 245.59430019948036],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array([-0.2549813451805683, 0.05058064413299489, 0.0007670480466289459, -0.007931492476075245, 0.0])

        # AprilTag size in meters (you'll need to adjust this based on your tag size)
        self.tag_size = 0.065  # 6.5cm tags

        # Initialize the AprilTag detector with options
        try:
            options = apriltag.DetectorOptions(families="tag36h11")  # Common tag family for Duckietown
            self.detector = apriltag.Detector(options)
            rospy.loginfo("AprilTag Detector initialized successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize AprilTag Detector: {e}")
            raise

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("AprilTag Publisher Node Started (Duckietown)")

    def shutdown(self):
        """Handle node shutdown"""
        rospy.loginfo("Shutting down AprilTag publisher node...")
        self.running = False

    def publish_camera_transform(self, stamp):
        """Publish the static transform from robot frame to camera frame"""
        if not self.running:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.robot_name
        transform.child_frame_id = "camera_optical_frame"
        # Set the transform (you might need to adjust these values based on your robot's configuration)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransformMessage(transform)

    def generate_message_from_pose(self, tag_id, pose):
        """Generate a message based on the tag ID and pose"""
        # First, handle the specific tag IDs with predefined messages
        if tag_id == 2:
            return "lucky says to yield"
        
        elif tag_id == 74:
            return "lucky says to wait for the traffic light"
        
        elif tag_id == 6:
            return "lucky says to go right"
        
        elif tag_id == 7:
            return "lucky says to go left"
        
        elif tag_id == 20:
            return "lucky says to stop"
        
        elif tag_id == 96:
            return "lucky says to slow down"
        
        # For other tags, generate messages based on pose
        # Get position and orientation values
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        # Calculate distance from the camera
        distance = math.sqrt(x*x + y*y + z*z)
        
        # Calculate horizontal angle (in degrees)
        angle_horizontal = math.degrees(math.atan2(y, x))
        
        # Generate messages based on tag_id and pose
        if tag_id == 0:
            if distance < 0.3:
                return f"Tag 0 is very close: {distance:.2f}m"
            elif distance < 0.8:
                return f"Tag 0 at medium range: {distance:.2f}m"
            else:
                return f"Tag 0 is far away: {distance:.2f}m"
        
        elif tag_id == 1:
            if abs(angle_horizontal) < 10:
                return f"Tag 1 centered (angle: {angle_horizontal:.1f}°)"
            elif angle_horizontal > 0:
                return f"Tag 1 to the left (angle: {angle_horizontal:.1f}°)"
            else:
                return f"Tag 1 to the right (angle: {angle_horizontal:.1f}°)"
        
        # Default message for other tags
        else:
            return f"Tag {tag_id} detected at distance: {distance:.2f}m, angle: {angle_horizontal:.1f}°"

    def image_callback(self, msg):
        """
        Callback function for the camera image topic.
        Processes the image to detect AprilTags and publishes the results.
        """
        if not self.running:
            return

        # Get the current time once for all messages
        current_time = rospy.Time.now()
        
        # Publish the camera transform
        self.publish_camera_transform(current_time)

        try:
            # Convert the compressed image data to a NumPy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode the image data using OpenCV
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"Error decoding image: {e}")
            return

        # Convert the image to grayscale for AprilTag detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Detect AprilTags in the grayscale image
        try:
            detections = self.detector.detect(gray)
            if detections:
                rospy.loginfo(f"Detected {len(detections)} AprilTags")
        except Exception as e:
            rospy.logerr(f"Error detecting AprilTags: {e}")
            return

        # Create an AprilTagDetectionArray message
        detection_array = AprilTagDetectionArray()

        # Iterate through the detected AprilTags
        for det in detections:
            detection = AprilTagDetection()
            detection.tag_id = int(det.tag_id)  # Store the tag ID as an integer
            detection.tag_family = det.tag_family.decode('utf-8')  # e.g., 'tag36h11'
            detection.hamming = det.hamming  # Hamming distance
            detection.decision_margin = det.decision_margin  # Detection confidence
            # Store the corner points of the tag
            detection.corners = [det.corners[0][0], det.corners[0][1],
                               det.corners[1][0], det.corners[1][1],
                               det.corners[2][0], det.corners[2][1],
                               det.corners[3][0], det.corners[3][1]]
            # Store the center point of the tag
            detection.center = [det.center[0], det.center[1]]
            # Populate homography
            detection.homography = det.homography.flatten().tolist()
            detection_array.detections.append(detection)

            # Estimate pose
            try:
                # Convert corners to numpy array
                corners = np.array(det.corners, dtype=np.float32)
                
                # Estimate pose using solvePnP
                obj_points = np.array([
                    [-self.tag_size/2, -self.tag_size/2, 0],
                    [self.tag_size/2, -self.tag_size/2, 0],
                    [self.tag_size/2, self.tag_size/2, 0],
                    [-self.tag_size/2, self.tag_size/2, 0]
                ])
                
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    corners,
                    self.camera_matrix,
                    self.dist_coeffs
                )
                
                if success:
                    # Convert rotation vector to rotation matrix
                    R, _ = cv2.Rodrigues(rvec)
                    
                    # Create pose message
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = current_time
                    pose_msg.header.frame_id = "camera_optical_frame"
                    
                    # Set position
                    pose_msg.pose.position.x = tvec[0][0]
                    pose_msg.pose.position.y = tvec[1][0]
                    pose_msg.pose.position.z = tvec[2][0]
                    
                    # Convert rotation matrix to quaternion
                    pose_msg.pose.orientation.w = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
                    pose_msg.pose.orientation.x = (R[2,1] - R[1,2]) / (4 * pose_msg.pose.orientation.w)
                    pose_msg.pose.orientation.y = (R[0,2] - R[2,0]) / (4 * pose_msg.pose.orientation.w)
                    pose_msg.pose.orientation.z = (R[1,0] - R[0,1]) / (4 * pose_msg.pose.orientation.w)
                    
                    # Publish the pose
                    if self.running:
                        self.pose_pub.publish(pose_msg)
                        rospy.loginfo(f"Published pose for tag {det.tag_id}")
                    
                    # Generate and publish message based on pose
                    message = self.generate_message_from_pose(det.tag_id, pose_msg.pose)
                    if self.running:
                        self.message_pub.publish(message)
                        rospy.loginfo(f"Message: {message}")
                        
                        # Print specific messages for the requested tag IDs
                        if det.tag_id == 2:
                            print("lucky says to yield")
                        elif det.tag_id == 74:
                            print("lucky says to wait for the traffic light")
                        elif det.tag_id == 6:
                            print("lucky says to go right")
                        elif det.tag_id == 7:
                            print("lucky says to go left")
                    
                    # Try to transform the pose to the robot's frame
                    try:
                        # Wait for transform
                        self.tf_listener.waitForTransform(self.robot_name, "camera_optical_frame", current_time, rospy.Duration(1.0))
                        # Transform the pose
                        pose_transformed = self.tf_listener.transformPose(self.robot_name, pose_msg)
                        rospy.loginfo(f"Tag {det.tag_id} pose in robot frame:{pose_transformed.pose}")
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                        rospy.logwarn(f"Could not transform pose to robot frame: {e}")
                    
            except Exception as e:
                rospy.logerr(f"Error estimating pose for tag {det.tag_id}: {e}")

        # Publish the AprilTag detection array
        if self.running: #check it is running by itself to avoid error at the end 
            self.pub.publish(detection_array)
            rospy.loginfo(f"Published {len(detections)} detections")

if __name__ == "__main__":
    try:
        node = AprilTagPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down AprilTag publisher node.")
