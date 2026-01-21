from pymycobot.mycobot import MyCobot
import math
import time
import numpy as np

camera_port = '/dev/ttyUSB0'
baudrate = '1000000'

bot = MyCobot(camera_port, baudrate)

for _ in range(5):
    if bot.is_controller_connected():
        break
    time.sleep(1)

if not bot.is_controller_connected():
    raise RuntimeError("Bot connection was unsuccessful.")
else:
    print("Bot connection was successful.")
    
    
    
traj = np.linspace([0,45,-130,0,0,-45],[70,0,-75,0,0,25],11)
print(bot.sync_send_angles(traj[0],50))
bot.set_gripper_state(0,100)
time.sleep(2)
for seg in traj:
    print(seg)
    
    status = False
    while not status:
        status = status = bot.sync_send_angles(seg,50)
        print(f"move to {seg},control_status = {status}")
    
    #time.sleep(0.1)
