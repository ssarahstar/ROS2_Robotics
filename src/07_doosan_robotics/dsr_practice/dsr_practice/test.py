import time
from onrobot import RG

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

#그리퍼 열기
gripper.open_gripper()

#그리퍼의 상태를 체크하고 동작 중이면 wait(동작이 끝날 때까지 기다리기)
while gripper.get_status()[0]:
    time.sleep(0.5)

    #그리퍼 닫기
    gripper.close_gripper()
while gripper.get_status()[0]:
    time.sleep(0.5)