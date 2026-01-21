import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json

"""
SceneNode -> send bot current scene
PoseNode -> send bot current Pose Information
ControlNode -> get bot Control Information

"""
class SceneNode(Node):
    def __init__(self, callback = None):
        super().__init__("SceneNode")
        self.publisher = self.create_publisher(
            Image,
            "SceneNode",
            10
        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


        if not self.cap.isOpened():
            # raise RuntimeError("Cannot open camera")
            None
        self.timer = self.create_timer(1.0 / 30.0, callback if callback else self.default_callback)
        self.get_logger().info("SceneNode is ready")

    def default_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to grab frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"

        self.publisher.publish(msg)

class PoseNode(Node):
    def __init__(self, bot, callback = None):
        super().__init__("PoseNode")
        
        self.bot = bot
        self.publisher = self.create_publisher(
            String,
            "PoseNode",
            10
        )

        self.timer = self.create_timer(0.1, callback if callback else self.default_callback)
        self.angles_timer = self.create_timer(0.1, self.angles_callback)
        self.coords_timer = self.create_timer(0.1, self.coords_callback)

        
        self.get_logger().info("PoseNode is ready")

        self.angles = None
        self.coords = None
    
    def angles_callback(self):
        self.angles = self.bot.get_angles()
    
    def coords_callback(self):
        self.coords = self.bot.get_coords()

    def default_callback(self):
        if not self.angles or not self.coords:
            return

        data = String()
        data.data = json.dumps(
            {
                "angle":self.angles,
                "coord":self.coords
            }
        )

        self.publisher.publish(data)


class ControlNode(Node):
    def __init__(self, bot,callback = None):
        super().__init__("Control_Node")
        
        self.bot = bot

        self.subscriber = self.create_subscription(
            String,
            "ControlNode",
            callback if callback else self.default_callback,
            10
        )

        self.get_logger().info("ControlNode is ready")

    def default_callback(self,msg):
        data = json.loads(msg.data)
            
        if data["tag"] == "Angle":
            id = data["id"]
            angle = data["angle"]
            self.bot.sync_send_angle(
                    id,
                    angle,
                    50
            )

        elif data["tag"] == "Angles":
            angles = data["angle"]
            self.bot.sync_send_angles(
                angles,
                50
            )

        elif data["tag"] == "Coord":
            id = data["id"]
            coord = data["coord"]
            self.bot.sync_send_coord(
                id,
                coord,
                50
            )

        elif data["tag"] == "Coords":
            coords = data["coord"]
            self.bot.sync_send_coords(
                coords,
                50
            )
        
        elif data["tag"] == "Gripper":
            grip_value = data["gripper"]
            self.bot.set_gripper_value(
                grip_value,
                50
            )
            
