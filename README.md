# 25_HC284
인간의 움직임을 실시간으로 인식하며 로봇팔과 주행 플랫폼을 동시에 제어하는 재난 구조용 협업 로봇 시스템

# 🤖 [25_HC284] 인간-로봇 협업형 웨어러블 재난 구조 메카닉

---

## 💡 1. 프로젝트 개요

### 1-1. 프로젝트 소개
- **프로젝트 명:** 인간–로봇 협업형 재난 구조 메카닉  
- **프로젝트 정의:**  
  인간의 움직임을 실시간으로 인식하고 이를 로봇의 손·팔·주행부에 정밀하게 반영함으로써  
  사람과 로봇이 동시에 협력할 수 있는 재난 대응형 협동 로봇 시스템

---

### 1-2. 개발 배경 및 필요성
최근 10년간, 전 세계는 기후 위기·초고층화·인프라 노후화로 인해 복합 재난이 급증하고 있습니다.  
이러한 환경에서 인간이 직접 접근하기 어려운 재난 현장(붕괴, 화재, 유독가스 등)에서는  
기존의 인력 중심 구조 방식이 접근성·안전성·지속성 측면에서 한계를 보이고 있습니다.  

이에 따라 로봇의 물리적 내구성과 정밀 제어 능력,  
인간의 상황 판단력과 유연한 의사 결정 능력을 결합한  
‘인간–로봇 협업형 구조 메카닉’의 필요성이 대두되었습니다.  

이 시스템은 단순 원격 조작형 로봇을 넘어,  
**착용형·자율형·협업형 구조 플랫폼**으로 진화함으로써  
재난 대응의 효율성과 생존율을 극대화하는 것을 목표로 합니다.

---

### 1-3. 프로젝트 특장점
- **정밀 제어 기반 구동 시스템:**  
  19자유도 로봇 손과 6축 로봇 팔에 고해상도 엔코더 및 PID·임피던스 제어를 적용하여,  
  미세한 움직임과 외력에도 안정적으로 반응하는 정밀 제어 구현  
- **웨어러블 연동 협동 제어:**  
  IMU·EMG·포지션 센서를 탑재한 글러브를 통해 사용자의 손·팔 동작을 실시간으로 로봇에 매핑  
- **자율 주행형 모빌리티 플랫폼:**  
  2D LiDAR·IMU·Depth 카메라 기반 SLAM 기술을 적용해 자율 이동 및 장애물 회피 가능  
- **ROS 기반 통합 제어 프레임워크:**  
  손·팔·주행부의 동작을 ROS 환경에서 통합 동기화하여 시뮬레이션·궤적 계획·충돌 회피 수행 가능  

---

### 1-4. 주요 기능
- **웨어러블 입력 장치:** IMU, 포지션, EMG 센서 기반 동작 인식 및 데이터 전송  
- **로봇 손(19 DoF):** 서보모터 직접 구동, 엔코더 피드백 기반 정밀 파지 제어  
- **로봇 팔(6 DoF):** 역기구학 기반 자세 계산 및 폐루프 궤적 제어  
- **자율 주행 플랫폼:** LiDAR + SLAM 기반 환경 인식 및 동적 이동  
- **통합 제어 시스템:** ROS 기반 실시간 동기화 및 모듈 간 협조 제어 구조  

---

### 1-5. 기대 효과 및 활용 분야
- **기대 효과:**  
  정밀 제어·자율 주행·실시간 협동 제어를 통합하여 인간과 로봇이 함께 움직이는  
  지능형 협업 플랫폼 구현 → 재난 대응 로봇의 정밀성·안정성·확장성 향상  
- **활용 분야:**  
  재난 구조 / 산업 자동화 / 의료·재활 / 교육·연구 / 국방·탐사 등  

---

### 1-6. 기술 스택
| 구분 | 사용 기술 |
|------|-------------|
| **통신** | CAN, UART, TCP/IP |
| **모터제어** | PID, 전류 기반 위치 제어 |
| **MCU 제어** | Timer, Interrupt |
| **Vision** | YOLOv11, OpenCV, RealSense SDK |
| **ROS** | URDF, Gazebo |
| **로봇팔 제어** | FK, IK, KDL |
| **센서 데이터** | 칼만 필터, 저역 통과 필터 |
| **배포 및 관리** | GitHub, STM32CubeIDE, ROS |

---

## 👥 2. 팀원 소개


<table>
  <tr>
    <td align="center" width="200" height="230" valign="top">
      <img src="https://github.com/user-attachments/assets/16435f88-e7d3-4e45-a128-3d32648d2d84" width="180" style="border-radius:10px; margin-bottom:10px;"><br>
      <b>안정은</b><br>
      팀장<br>
      • 손 센서부&구동부<br>설계 보조<br>• 프로젝트 관리
    </td>
    <td align="center" width="200" height="230" valign="top">
      <img src="https://github.com/user-attachments/assets/16435f88-e7d3-4e45-a128-3d32648d2d84" width="180" style="border-radius:10px; margin-bottom:10px;"><br>
      <b>최민권</b><br>
      멘티<br>
      • 센서부·구동부 기획 <br>• 설계 총괄 <br>• 시스템 제어&기능 구현
    </td>
    <td align="center" width="200" height="230" valign="top">
      <img src="https://github.com/user-attachments/assets/16435f88-e7d3-4e45-a128-3d32648d2d84" width="180" style="border-radius:10px; margin-bottom:10px;"><br>
      <b>장형준</b><br>
      멘티<br>
      • 구동부 설계  <br>• 서류 작업 보조 <br>• 모바일 기획 보조
    </td>
    <td align="center" width="200" height="230" valign="top">
      <img src="https://github.com/user-attachments/assets/16435f88-e7d3-4e45-a128-3d32648d2d84" width="180" style="border-radius:10px; margin-bottom:10px;"><br>
      <b>김서희</b><br>
      멘티<br>
      • 구동부 설계 보조  <br>• 회로 구성 보조 <br>• 제작
    </td>
    <td align="center" width="200" height="230" valign="top">
      <img src="https://github.com/user-attachments/assets/16435f88-e7d3-4e45-a128-3d32648d2d84" width="180" style="border-radius:10px; margin-bottom:10px;"><br>
      <b>김동영</b><br>
      멘토<br>
      • 프로젝트 멘토 <br>• 기술 자문 <br>• 프로젝트 관리
    </td>
  </tr>
</table>



---

## ⚙️ 3. 시스템 구성도
- **서비스 구성도**  
  (시스템 블록도, 데이터 흐름도, 제어 구조도 등 첨부 예정)]
  
<img width="1685" height="956" alt="image" src="https://github.com/user-attachments/assets/b0f8f22a-bd32-4aca-95cd-5951eeefd84f" />

- **엔티티 관계도(ERD)**  
  (DB 설계 다이어그램 첨부)

---

## 🎬 4. 작품 소개영상
아래와 같이 작성하면, 썸네일과 유튜브 영상 링크를 함께 등록할 수 있습니다.  
예시:  
[![프로젝트 소개](https://www.youtube.com/watch?v=w8o6nSqm-DI)

---

## 💻 5. 핵심 소스코드
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
import kdl_parser_py.urdf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
from std_msgs.msg import Bool

class KDL_IK_Solver:
    def __init__(self):
        rospy.init_node("kdl_ik_solver", anonymous=True)
        self.cmd_pub = rospy.Publisher(
            "/arm_controller/command", JointTrajectory, queue_size=10
        )

        base_link = rospy.get_param("~base_link", "base_link")
        tip_link  = rospy.get_param("~tip_link", "palm_1")
        self.max_speed = rospy.get_param("~max_speed", 1.5)

        # rospy.loginfo("Loading URDF from parameter server...")
        robot = URDF.from_parameter_server()

        success, self.kdl_tree = kdl_parser_py.urdf.treeFromUrdfModel(robot)
        if not success:
            # rospy.logerr("Failed to parse URDF into KDL tree.")
            return

        self.chain = self.kdl_tree.getChain(base_link, tip_link)
        # rospy.loginfo_once(
        #     f"Created KDL chain from [{base_link}] to [{tip_link}] "
        #     f"with {self.chain.getNrOfJoints()} joints."
        # )

        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)

        self.joint_names = []
        for i in range(self.chain.getNrOfSegments()):
            joint = self.chain.getSegment(i).getJoint()
            jtype = str(joint.getType())
            if "None" not in jtype:
                self.joint_names.append(joint.getName())
                
        self.joint_names.pop()
        # rospy.loginfo(f"Using joints: {self.joint_names}")

        self.q_current = kdl.JntArray(len(self.joint_names))
        
        self.switch_on = False

        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/fusion_pose", PoseStamped, self.fusion_pose_callback)
        rospy.Subscriber("/switch", Bool, self.switch_callback)

        # rospy.loginfo("KDL IK Solver node initialized.")
        rospy.spin()
        
        
    def switch_callback(self, msg):
        self.switch_on = msg.data  
        rospy.loginfo_throttle(1.0, f"switch status {self.switch_on}")


    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q_current[i] = msg.position[idx]

    def fusion_pose_callback(self, msg):
        if not self.switch_on:
            rospy.loginfo_throttle(1.0, "switch off")
            return
        
        # start = time.time()
        
        if all(abs(self.q_current[i]) < 1e-6 for i in range(self.q_current.rows())):
            rospy.logwarn_throttle(1.0, "No valid joint_states yet — skipping IK.")
            return

        pos = msg.pose.position
        ori = msg.pose.orientation

        target_frame = kdl.Frame(
            kdl.Rotation.Quaternion(ori.x, ori.y, ori.z, ori.w),
            kdl.Vector(pos.x, pos.y, pos.z)
        )

        q_sol = kdl.JntArray(len(self.joint_names))
        ret = self.ik_solver.CartToJnt(self.q_current, target_frame, q_sol)
        
        deltas = [abs(float(q_sol[i] - self.q_current[i])) for i in range(q_sol.rows())]
        max_delta = max(deltas) if deltas else 0.0
        speed = max(self.max_speed, 1e-6)
        duration = max_delta / speed

        duration = max(duration, 0.1)

        if ret >= 0:
            joint_values = [round(q_sol[i], 4) for i in range(q_sol.rows())]
            # rospy.loginfo_throttle(1.0, f" IK Solved: {joint_values}")
            
            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = joint_values
            point.time_from_start = rospy.Duration(duration)
            traj.points.append(point)
            self.cmd_pub.publish(traj)
            # end = time.time()
            # print(f"연산 시간time : {end-start}")
        else:
            rospy.logwarn_throttle(1.0, "IK failed")

if __name__ == "__main__":
    KDL_IK_Solver()

