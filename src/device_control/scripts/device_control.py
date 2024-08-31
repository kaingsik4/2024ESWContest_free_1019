#!/usr/bin/env python

import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList

# 모터 ID와 Load 임계값 설정
DXL_ID_1 = 1  # 첫 번째 Dynamixel 모터 ID
DXL_ID_2 = 2  # 두 번째 Dynamixel 모터 ID
LOAD_THRESHOLD = 90  # 부하 임계값 (적절히 조정 필요)

# 현재 부하를 저장하는 전역 변수
current_load_1 = 0
current_load_2 = 0

def dynamixel_state_callback(msg):
    global current_load_1, current_load_2
    # Dynamixel 상태 리스트에서 현재 부하 값 추출
    for state in msg.dynamixel_state:
        if state.id == DXL_ID_1:
            current_load_1 = state.present_load
            rospy.loginfo("Current Load of Motor 1: %d", current_load_1)
        elif state.id == DXL_ID_2:
            current_load_2 = state.present_load
            rospy.loginfo("Current Load of Motor 2: %d", current_load_2)

def set_goal_position(client, motor_id, goal_position):
    try:
        # 모터의 목표 위치 설정
        response = client('', motor_id, 'Goal_Position', goal_position)
        if response.comm_result:
            rospy.loginfo("Motor ID %d: Goal Position set to %d", motor_id, goal_position)
        else:
            rospy.logwarn("Failed to set goal position for motor ID %d", motor_id)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def main():
    rospy.init_node('dynamixel_control_node', anonymous=True)

    # DynamixelCommand 서비스 클라이언트 준비
    rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
    dynamixel_command_client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)

    # 모터 상태 구독
    rospy.Subscriber('/dynamixel_workbench/dynamixel_state', DynamixelStateList, dynamixel_state_callback)

    # 모터 1과 2의 초기 목표 위치 설정
    set_goal_position(dynamixel_command_client, DXL_ID_1, 0)
    set_goal_position(dynamixel_command_client, DXL_ID_2, 0)

    motor1_position = 0.0
    motor2_position = 0.0

    # 목표 위치와 Load 제어 루프
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        # Load 제어
        if current_load_1 < LOAD_THRESHOLD:
            motor1_position += 0.1
        else:
            pass
        if current_load_2 < LOAD_THRESHOLD:
            motor2_position += 0.1
        else:
            pass
        # 모터 1 제어
        if current_load_1 < LOAD_THRESHOLD:
            rospy.loginfo("Motor 1 Load is under threshold, moving motor...")
            set_goal_position(dynamixel_command_client, DXL_ID_1, motor1_position)
        else:
            rospy.loginfo("Motor 1 Load threshold reached. Stopping motor 1.")

        # 모터 2 제어
        if current_load_2 < LOAD_THRESHOLD:
            rospy.loginfo("Motor 2 Load is under threshold, moving motor...")
            set_goal_position(dynamixel_command_client, DXL_ID_2, motor2_position)
        else:
            rospy.loginfo("Motor 2 Load threshold reached. Stopping motor 2.")
            break

        rate.sleep()

    # 모터 토크 비활성화
    try:
        dynamixel_command_client('', DXL_ID_1, 'Torque_Enable', 0)
        dynamixel_command_client('', DXL_ID_2, 'Torque_Enable', 0)
        rospy.loginfo("Motor 1 and 2 Torque Disabled")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
