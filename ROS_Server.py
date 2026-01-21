import subprocess
import threading as th
import time

from pymycobot.mycobot import MyCobot

from ROS_Node import SceneNode, PoseNode, ControlNode

import rclpy
from rclpy.executors import MultiThreadedExecutor

class ROS_Server:
    """
    parm:
    - server_process #rosbridge_server
    - bot #JetCobot


    func:
    - launch_rosbridge_server(port) #open a rosbridge server
    - connect_bot(bot_camera_bot, bot_baudrate) #connect a JetCobot
    
    
    """


    def __init__(
        self,
        ROS_BRIDGE_ENABLE,
        SERVER_PORT,
        BOT_CAMERA_PORT,
        BOT_BAUDRATE,
        SCENE_NODE_CALLBACK = None,
        POSE_NODE_CALLBACK = None,
        CONTROL_NODE_CALLBACK = None
    ):
        if ROS_BRIDGE_ENABLE:
            print(f"ROS_BRIDGE_ENABLE = TRUE, Try to open the rosbridge_server on SERVER_PORT = {SERVER_PORT} ")
            self.launch_rosbridge_server(SERVER_PORT)
        
        self.connect_bot(
            BOT_CAMERA_PORT,
            BOT_BAUDRATE
        )

        rclpy.init()

        self.scene_node = SceneNode(SCENE_NODE_CALLBACK)
        self.pose_node = PoseNode(
            self.bot,
            POSE_NODE_CALLBACK
        )
        self.control_node = ControlNode(
            self.bot,
            CONTROL_NODE_CALLBACK
        )

        executor = MultiThreadedExecutor(num_threads = 3)
        executor.add_node(self.scene_node)
        executor.add_node(self.pose_node)
        executor.add_node(self.control_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            self.scene_node.destroy_node()
            self.pose_node.destroy_node()
            self.control_node.destroy_node()
            rclpy.shutdown()



    def launch_rosbridge_server(self, port):     
        self.server_process = subprocess.Popen(
            [
                'ros2',
                'launch',
                'rosbridge_server',
                'rosbridge_websocket_launch.xml',
                f'port:={port}'
            ],       
            stdout = subprocess.PIPE,
            stderr = subprocess.PIPE,
            text = True
        )

        self.server_open_state = True

        def read_stdout():
            for line in self.server_process.stdout:
                print(f"server_msg:{line}")
                if "Errno" in line:
                    self.server_process.terminate()
                    
                    try:
                        raise RuntimeError(line)
                    except Exception as e:
                        self.server_open_state = False

        th.Thread(
            target = read_stdout,
            daemon = True
        ).start()

    def connect_bot(self, camera_port, baudrate):
        self.bot = MyCobot(camera_port, baudrate)

        for _ in range(5):
            if self.bot.is_controller_connected():
                break
            time.sleep(1)

        if not self.bot.is_controller_connected():
            raise RuntimeError("Bot connection was unsuccessful.")
        else:
            print("Bot connection was successful.")
