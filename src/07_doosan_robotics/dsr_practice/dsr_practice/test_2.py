from onrobot import RG
import time

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
gripper.move_gripper(width_val=400, force_val=10)
while gripper.get_status()[0]:
    time.sleep(0.5)

    #그리퍼의 현재 너비 출력
    print(f'get_width_with_offset: {gripper.get_width_with_offset()}')
    