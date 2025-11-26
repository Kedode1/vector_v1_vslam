import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from turtlesim.msg import Pose

# ============================================================
# ROS IMAGE → CV2 (no CvBridge)
# ============================================================
def ros_img_to_cv2(msg):
    """
    Convert ROS2 sensor_msgs.msg.Image to OpenCV image manually.
    Supports: bgr8, rgb8, mono8.
    """
    img_data = np.frombuffer(msg.data, dtype=np.uint8)

    if msg.encoding == 'rgb8':
        image = img_data.reshape((msg.height, msg.width, 3))
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    elif msg.encoding == 'bgr8':
        image = img_data.reshape((msg.height, msg.width, 3))
        return image

    elif msg.encoding == 'mono8':
        return img_data.reshape((msg.height, msg.width))

    else:
        raise ValueError(f"Unsupported encoding: {msg.encoding}")


# ============================================================
# CV2 → ROS IMAGE (no CvBridge)
# ============================================================
def cv2_to_ros_img(cv_image):
    msg = Image()
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = cv_image.shape[1] * 3
    msg.data = cv_image.tobytes()
    return msg

class VO_Node(Node):

    def __init__(self):
        super().__init__('vo_node')

        # ROS2 Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.encoder_sub = self.create_subscription(
            Pose,
            '/encoder_readings',
            self.encoder_callback,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/image_sift_tracker',
            10
        )

        # -----------------------------
        # VO INITIALIZATION
        # -----------------------------
        self.fx = self.fy = 700.0
        self.cx = 320.0
        self.cy = 240.0
        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0,       0,       1]
        ], dtype=np.float32)

        # SIFT + KLT params
        self.sift = cv2.SIFT_create(nfeatures=50)
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS |
                      cv2.TERM_CRITERIA_COUNT, 100, 0.01)
        )

        # thresholds
        self.MOTION_THRESHOLD = 0.8
        self.KF_MOTION_THRESHOLD = 2.5
        self.KF_MIN_FEATURES = 20
        self.KF_MAX_FRAMES = 20

        # VO state
        self.initialized = False
        self.prev_gray = None
        self.points = None
        self.frame_id = 0
        self.last_kf_id = 0
        self.encoder_distance = 0.0

        # Pose (4×4)
        self.T_global = np.eye(4)

        self.get_logger().info("Visual Odometry Node Initialized.")


    # --------------------------------------------------
    # ENCODER CALLBACK — updates distance traveled
    # --------------------------------------------------
    def encoder_callback(self, msg):
        # Expecting msg.data[0] = distance since last frame (meters)
        if msg.y > 0:
            self.encoder_distance = msg.y
        else:
            self.encoder_distance = 0.0


    # --------------------------------------------------
    # IMAGE CALLBACK — main VO pipeline
    # --------------------------------------------------
    def image_callback(self, msg):
        # Convert ROS Image -> OpenCV BGR
        frame = ros_img_to_cv2(msg)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.frame_id += 1

        # ----------------------------------------
        # INITIALIZE WITH SIFT FEATURES
        # ----------------------------------------
        if not self.initialized:
            kp = self.sift.detect(frame_gray, None)
            if len(kp) < 10:
                return

            self.points = np.array([k.pt for k in kp], dtype=np.float32).reshape(-1, 1, 2)
            self.prev_gray = frame_gray
            self.initialized = True
            self.get_logger().info(f"[INIT] Initialized with {len(self.points)} features.")
            return

        # ----------------------------------------
        # OPTICAL FLOW TRACKING
        # ----------------------------------------
        next_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            frame_gray,
            self.points,
            None,
            **self.lk_params
        )

        good_new = next_points[status == 1]
        good_old = self.points[status == 1]

        # ----------------------------------------
        # MOTION COMPUTATION
        # ----------------------------------------
        if len(good_new) > 0:
            disp = np.linalg.norm(good_new - good_old, axis=1)
            motion = np.mean(disp)
        else:
            motion = 0

        # ----------------------------------------
        # KEYFRAME DECISION
        # ----------------------------------------
        new_keyframe = (
            motion > self.KF_MOTION_THRESHOLD or
            len(good_new) < self.KF_MIN_FEATURES or
            (self.frame_id - self.last_kf_id) > self.KF_MAX_FRAMES
        )

        # ----------------------------------------
        # POSE ESTIMATION (E-MATRIX + ENCODER FUSION)
        # ----------------------------------------
        if new_keyframe and motion > self.MOTION_THRESHOLD and len(good_new) >= 8:

            E, mask = cv2.findEssentialMat(
                good_new, good_old, self.K,
                method=cv2.RANSAC,
                prob=0.999,
                threshold=1.0
            )

            if E is not None:
                _, R, t_vo, pose_mask = cv2.recoverPose(
                    E, good_new, good_old, self.K
                )

                # ---------------------
                # FUSE ENCODER SCALE
                # ---------------------
                t_vo = t_vo.reshape(3)
                if np.linalg.norm(t_vo) > 1e-6:
                    t_dir = t_vo / np.linalg.norm(t_vo)
                else:
                    t_dir = np.zeros(3)

                d_enc = self.encoder_distance        # meters
                t_scaled = t_dir * d_enc

                # build transform
                T_rel = np.eye(4)
                T_rel[:3, :3] = R
                T_rel[:3, 3] = t_scaled

                self.T_global = self.T_global @ T_rel

                pos = self.T_global[:3, 3]
                self.get_logger().info(
                    f"[POSE] X={pos[0]*1000:.1f} mm  "
                    f"Y={pos[1]*1000:.1f} mm  "
                    f"Z={pos[2]*1000:.1f} mm"
                )

                self.last_kf_id = self.frame_id

            # Re-detect fresh SIFT features
            kp = self.sift.detect(frame_gray, None)
            self.points = np.array([k.pt for k in kp], dtype=np.float32).reshape(-1, 1, 2)

        else:
            # continue optical flow tracking
            self.points = good_new.reshape(-1, 1, 2)

        # update previous frame
        self.prev_gray = frame_gray

        # ---------------------------------------------------------
        # DRAW FEATURES + PUBLISH ANNOTATED IMAGE
        # ---------------------------------------------------------
        annotated = frame.copy()
        for p in self.points:
            x, y = p.ravel()
            cv2.circle(annotated, (int(x), int(y)), 2, (0, 255, 0), -1)

        ros_annotated = cv2_to_ros_img(annotated)
        self.image_pub.publish(ros_annotated)


# --------------------------------------------------
# ENTRY POINT
# --------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = VO_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
