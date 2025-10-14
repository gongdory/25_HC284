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
