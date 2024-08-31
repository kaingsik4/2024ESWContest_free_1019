# 제22회 임베디드 소프트웨어 경진대회 공모전 제출 레포지토리

이곳은 **제22회 임베디드 소프트웨어 경진대회 공모전** 제출을 위한 레포지토리입니다.

## 프로젝트 개요

이 프로젝트는 임베디드 시스템 소프트웨어의 혁신적인 기능 구현과 성능 향상을 목표로 합니다. 

## 사용 방법

1. 패키지 클론 및 빌드:
    ```bash
    cd ~
    git clone https://github.com/kaingsik4/workspace.git
    cd ~/sw_embeded_ws/
    catkin_make
    ```

2. 실행:
    ```bash
    roslaunch dynamixel_workbench_controllers dynamixel_controller.launch
    rosrun device_control device_control_node.py
    ```

## 시스템 요구사항

- Ubuntu 20.04 LTS
- ROS Noetic
- Dynamixel SDK 및 Workbench

## 라이선스

이 프로젝트는 [MIT 라이선스](LICENSE)를 따릅니다.

## 문의

프로젝트에 대한 문의 사항은 [kaingsik4@gmail.com](mailto:kaingsik4@gmail.com)으로 연락주세요.

---

JungminRobotics | 2024
