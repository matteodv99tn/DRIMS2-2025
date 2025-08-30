import numpy as np
import rclpy
from rclpy.node import Node
from numpy.typing import NDArray
from typing import Optional, Tuple

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from drims2_msgs.srv import DiceIdentification
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation


def most_frequent(l: list[int]) -> int:
    return max(set(l), key=l.count)


class DiceDetector(Node):

    def __init__(self):
        super().__init__('dice_detector')

        # Bridge OpenCV <-> ROS
        self.bridge = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(
            Image, '/color/video/image', self.listener_callback, 10
        )

        # Publisher
        self.publisher = self.create_publisher(Image, '/dice_detector/circle', 10)

        # Service
        self.srv = self.create_service(
            DiceIdentification, 'dice_identification', self.handle_service
        )

        self.get_logger().info("Dice Detector node started")

        self.detected_face_id: Optional[int] = None
        self.detected_face_hist = list()

        # # Parameters of bags
        # self.intrinsics = np.array([
        #     [1.57855692e+03, 0.00000000e+00, 9.52440456e+02],
        #     [0.00000000e+00, 1.58246225e+03, 5.45655012e+02],
        #     [0.00000000e+00, 0.00000000e+00, 1.00000000e+00],
        # ],
        #                            dtype=np.float64)

        # # --- Distortion coefficients (k1, k2, p1, p2, k3)
        # self.dist_coeffs = np.array([
        #     0.07206577, 0.08106335, 0.00300317, 0.00042163, -0.40383728
        # ],
        #                             dtype=np.float64)

        # self.extrinsics = np.array([
        #     [0.998855, -0.019105, 0.043869, -0.236381],
        #     [0.020615, 0.999201, -0.034238, -0.214228],
        #     [-0.043180, 0.035103, 0.998450, 0.742136],
        #     [0.0, 0.0, 0.0, 1.0],
        # ],
        #                            dtype=np.float64)

        # Parameters

        self.intrinsics = np.array([
            [1.54588237e+03, 0.00000000e+00, 9.46934926e+02],
            [0.00000000e+00, 1.55502898e+03, 5.44959477e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00],
        ],
                                   dtype=np.float64)
        self.dist_coeffs = np.array([
            0.13503889, -0.28098708, 0.0003601, -0.00038133, 0.13643112
        ],
                                    dtype=np.float64)

        self.extrinsics = np.array([
            [0.999831, -0.015292, -0.010167, -0.062168],
            [0.015271, 0.999881, -0.002154, -0.071062],
            [0.010198, 0.001998, 0.999946, 0.717149],
            [0.0, 0.0, 0.0, 1.0],
        ],
                                   dtype=np.float64)

    def listener_callback(self, msg):
        # Convert ROS image -> OpenCV
        bgr_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Undistort
        bgr_frame = cv2.undistort(bgr_frame, self.intrinsics, self.dist_coeffs)

        bgr_frame_cropped = self.crop_image_to_dice(bgr_frame)
        if bgr_frame_cropped is None:
            self.get_logger().warn("Unable to crop frame to dice")
            return

        num_dots, dice_features = self.detect_dice_number(bgr_frame_cropped)
        self.detected_face_hist.append(num_dots)

        # Process face id every 10 images
        if len(self.detected_face_hist) == 10:
            face_id = most_frequent(self.detected_face_hist)
            if self.detected_face_hist.count(face_id) > 7:
                self.detected_face_id = face_id
            else:
                self.detected_face_id = None

            self.detected_face_hist = list()

        out_frame = dice_features

        # img_msg = self.bridge.cv2_to_imgmsg(out_frame, encoding='8UC1')
        img_msg = self.bridge.cv2_to_imgmsg(out_frame, encoding='8UC3')
        # img_msg = self.bridge.cv2_to_imgmsg(out_frame, encoding='bgr8')
        img_msg.header = msg.header
        self.publisher.publish(img_msg)

    def crop_image_to_dice(self, bgr_frame) -> Optional[NDArray]:
        hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        hsv_lb = np.array([20, 100, 100])
        hsv_ub = np.array([30, 255, 255])
        hsl_mask = cv2.inRange(hsv_frame, hsv_lb, hsv_ub)
        all_contours, _ = cv2.findContours(hsl_mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in all_contours if (cv2.contourArea(c) > 500)]

        if len(contours) != 1:
            self.get_logger().warn(f"Found {len(contours)} dice contours!")
            return None

        rect = cv2.minAreaRect(contours[0])
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        self.dice_angle = rect[2]
        self.dice_position = self.detect_dice_position(contours[0])
        print(self.dice_position)

        # To draw identified dice bounding box
        # cv2.drawContours(bgr_frame, [box], 0, (0, 0, 255), 2)

        # Crop image to dice
        width = int(rect[1][0])
        height = int(rect[1][1])
        src_pts = box.astype("float32")
        dst_pts = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1],
                            [0, height - 1]],
                           dtype="float32")
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        bgr_frame_cropped = cv2.warpPerspective(bgr_frame, M, (width, height))
        return bgr_frame_cropped

    def detect_dice_number(self, dice_frame: NDArray) -> Tuple[int, NDArray]:
        gray_frame = cv2.cvtColor(dice_frame, cv2.COLOR_BGR2GRAY)
        params = cv2.SimpleBlobDetector_Params()

        # Filter by area (size of the blobs)
        params.filterByArea = True
        params.minArea = 40
        params.maxArea = 500

        # Filter by inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.4
        params.maxInertiaRatio = 1.0

        # Detect and draw
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(gray_frame)
        frame_with_features = cv2.drawKeypoints(
            gray_frame,
            keypoints,
            None, (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )
        return len(keypoints), frame_with_features

    def detect_dice_position(self, contour: NDArray) -> np.ndarray:
        moments = cv2.moments(contour)
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        K = self.intrinsics
        Kinv = np.linalg.inv(K)
        H = self.extrinsics
        Hinv = np.linalg.inv(H)

        depth = (H @ np.array([0.0, 0.0, 0.0, 1.0]))[2]
        depth *= 1000.0

        norm_pose = Kinv @ np.array([cx, cy, 1])
        pos_in_cf = np.hstack([norm_pose * depth * 0.001, 1])
        pose = Hinv @ pos_in_cf
        return pose[0:3]

    def handle_service(self, request, response):
        # Here you put your detection values
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "checkerboard"

        a = np.deg2rad(self.dice_angle)
        Rz = np.array([
            [np.cos(a), -np.sin(a), 0.0],
            [np.sin(a), np.cos(a), 0.0],
            [0.0, 0.0, 1.0],
        ])
        Rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

        rot = Rz @ Rx
        q = Rotation.from_matrix(rot).as_quat()

        if self.detected_face_id is None:
            response.success = False
            return response

        pose.pose.position.x = self.dice_position[0]
        pose.pose.position.y = self.dice_position[1]
        pose.pose.position.z = self.dice_position[2]

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        response.pose = pose
        response.success = True
        response.face_number = self.detected_face_id
        self.detected_face_id = None

        self.get_logger().info("Service called -> returning static dice info")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DiceDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
