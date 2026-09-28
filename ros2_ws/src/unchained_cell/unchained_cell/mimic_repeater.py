"""
Publish the AG-145's MIMIC joint values so rviz renders the true gripper.

The mock/hardware controllers publish only the actuated
`gripper_finger1_joint`; the other seven gripper joints are URDF mimics.
robot_state_publisher does NOT resolve mimics — it leaves unpublished
joints at their spawn pose, so rviz shows a broken half-open gripper no
matter what the sim is actually doing (move_group resolves mimics
internally, so collision checking was always correct — only the DISPLAY
lied). This tiny node closes that gap: it listens to /joint_states,
computes every mimic from finger1, and republishes them on /joint_states
(robot_state_publisher merges per-joint by latest message).

Multipliers from urdf/dh_ag145_macro.xacro.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

MIMICS = {
    "gripper_finger2_joint": 1.0,
    "gripper_finger1_finger_joint": 0.5,
    "gripper_finger2_finger_joint": 0.5,
    "gripper_finger1_inner_knuckle_joint": 1.49,
    "gripper_finger2_inner_knuckle_joint": 1.49,
    "gripper_finger1_finger_tip_joint": 1.49,
    "gripper_finger2_finger_tip_joint": 1.49,
}
SOURCE = "gripper_finger1_joint"


class MimicRepeater(Node):
    def __init__(self):
        super().__init__("gripper_mimic_repeater")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.sub = self.create_subscription(JointState, "/joint_states",
                                            self.cb, 10)
        self.last = None
        # publish continuously (10 Hz): robot_state_publisher and any
        # late-joining rviz must keep receiving the mimic values — a
        # publish-on-change one-shot is missed by late subscribers.
        self.timer = self.create_timer(0.1, self.tick)

    def cb(self, msg):
        if SOURCE not in msg.name:
            return  # our own republication or unrelated publisher
        self.last = msg.position[msg.name.index(SOURCE)]

    def tick(self):
        if self.last is None:
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(MIMICS.keys())
        out.position = [m * self.last for m in MIMICS.values()]
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(MimicRepeater())


if __name__ == "__main__":
    main()
