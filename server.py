import argparse
from ROS_Server import ROS_Server as RS


arg = argparse.ArgumentParser()

arg.add_argument(
    '--bridge_server_enable',
    type = bool,
    default = False,
    help = 'use rosbridge server mode'
)

arg.add_argument(
    '--server_port',
    type = int,
    default = 9090,
    help = 'rosbridge server port'
)

arg.add_argument(
    '--bot_camera_port',
    type = str,
    default = '/dev/ttyUSB0',
    help = 'JetCobot camera port'
)

arg.add_argument(
    '--bot_baudrate',
    type = str,
    default = '1000000',
    help = 'JetCobot baudrate'
)

arg = arg.parse_args()

server = RS(
    ROS_BRIDGE_ENABLE = arg.bridge_server_enable,
    SERVER_PORT = arg.server_port,
    BOT_CAMERA_PORT = arg.bot_camera_port,
    BOT_BAUDRATE = arg.bot_baudrate
)
