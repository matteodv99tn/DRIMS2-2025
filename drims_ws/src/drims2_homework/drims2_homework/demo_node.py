import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from drims2_motion_server.motion_client import MotionClient
from drims2_msgs.srv import DiceIdentification
from moveit_msgs.msg import MoveItErrorCodes
from typing import Tuple
from tf2_ros import StaticTransformBroadcaster

REAL = True

if REAL:
    GRIPPER_ACTION_NAME = '/robotiq_2f_urcap_adapter/robotiq_hande_urcap_adapter/gripper_command'
else:
    GRIPPER_ACTION_NAME = '/gripper_action_controller/gripper_cmd'
    
GRIPPER_ACTION_NAME = '/gripper_action_controller/gripper_cmd'

# GRIPPER_ACTION_NAME = '/gripper_action_controller/gripper_cmd'
# GRIPPER_ACTION_NAME = '/robotiq_2f_urcap_adapter/robotiq_hande_urcap_adapter/gripper_command'


class VisionClientNode(Node):

    def __init__(self):
        super().__init__('vision_client_node', use_global_arguments=False)
        self.dice_identification_client = self.create_client(
            DiceIdentification, 'dice_identification')

    def dice_identification(self) -> Tuple[int, PoseStamped, bool]:
        if not self.dice_identification_client.wait_for_service(
                timeout_sec=5.0):
            raise RuntimeError("DiceIdentification service not available")

        request = DiceIdentification.Request()

        result_future = self.dice_identification_client.call_async(request)
        rclpy.spin_until_future_complete(self, result_future)

        face_number = result_future.result().face_number
        pose = result_future.result().pose
        success = result_future.result().success

        return face_number, pose, success


def main():
    rclpy.init()

    # MotionClient is already a Node, so we can use its logger directly
    motion_client_node = MotionClient(gripper_action_name=GRIPPER_ACTION_NAME)
    vision_client_node = VisionClientNode()
    demo_node = rclpy.create_node(node_name="demo_node",
                                  use_global_arguments=False)
    tf_broadcaster = StaticTransformBroadcaster(demo_node)

    logger = demo_node.get_logger()

    # --- 0) Move to home configuration and open gripper---
    home_joints = np.deg2rad([86.0, -100.0, 136.0, -143.0, -90.0, 0.0]).tolist()

    logger.info("Moving to home configuration...")
    result = motion_client_node.move_to_joint(home_joints)
    logger.info(f"Home reached: {result.val}")
    reached_goal, stalled = motion_client_node.gripper_command(
        position=0.045)  # Open gripper

    if result.val != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach home configuration: {result.val}")
        motion_client_node.destroy_node()
        vision_client_node.destroy_node()
        demo_node.destroy_node()
        rclpy.shutdown()
        return 0

    while True:

        logger.info("=================== New tentative")
        pick_pose = PoseStamped()

        # # 3.1 Drop the dice
        # pick_pose.header.frame_id = "checkerboard"
        # pick_pose.header.stamp = demo_node.get_clock().now().to_msg()
        # pick_pose.pose.position.x = 0.0
        # pick_pose.pose.position.y = 0.0
        # pick_pose.pose.position.z = 0.0
        # pick_pose.pose.orientation.x = 0.0
        # pick_pose.pose.orientation.y = 0.0
        # pick_pose.pose.orientation.z = 0.0
        # pick_pose.pose.orientation.w = 1.0

        # logger.info("Motion 3.1")
        # result = motion_client_node.move_to_pose(pick_pose,
        #                                          cartesian_motion=False)
        # if result.val != MoveItErrorCodes.SUCCESS:
        #     logger.error(f"Failed motion 3.1: {result.val}")
        #     motion_client_node.destroy_node()
        #     vision_client_node.destroy_node()
        #     demo_node.destroy_node()
        #     rclpy.shutdown()
        #     return 0

        # 1) Identify
        face_id, dice_pose, success = vision_client_node.dice_identification()
        tf_msg = TransformStamped()
        tf_msg.header = dice_pose.header
        tf_msg.child_frame_id = "dice_tf"
        tf_msg.transform.translation.x = dice_pose.pose.position.x
        tf_msg.transform.translation.y = dice_pose.pose.position.y
        tf_msg.transform.translation.z = dice_pose.pose.position.z
        tf_msg.transform.rotation.x = dice_pose.pose.orientation.x
        tf_msg.transform.rotation.y = dice_pose.pose.orientation.y
        tf_msg.transform.rotation.z = dice_pose.pose.orientation.z
        tf_msg.transform.rotation.w = dice_pose.pose.orientation.w
        tf_broadcaster.sendTransform(tf_msg)

        if not success or face_id == 1:
            logger.error('Dice identification failed or finished')
            motion_client_node.destroy_node()
            vision_client_node.destroy_node()
            demo_node.destroy_node()
            rclpy.shutdown()
            return 0

        # 2) Grasp
        # logger.info(f'Dice position {dice_pose}')

        # 2.1 go above
        pick_pose.header.frame_id = "dice_tf"
        pick_pose.header.stamp = demo_node.get_clock().now().to_msg()
        pick_pose.pose.position.x = 0.0
        pick_pose.pose.position.y = 0.0
        pick_pose.pose.position.z = 0.10
        pick_pose.pose.orientation.x = 1.0
        pick_pose.pose.orientation.y = 0.0
        pick_pose.pose.orientation.z = 0.0
        pick_pose.pose.orientation.w = 0.0
        logger.info(f"Sent pose: {pick_pose}")

        logger.info("Motion 2.1")
        result = motion_client_node.move_to_pose(pick_pose,
                                                 cartesian_motion=False)
        if result.val != MoveItErrorCodes.SUCCESS:
            logger.error(f"Failed motion 2.1: {result.val}")
            motion_client_node.destroy_node()
            vision_client_node.destroy_node()
            demo_node.destroy_node()
            rclpy.shutdown()
            return 0

        # 2.2 go down
        pick_pose.pose.position.z = 0.0
        logger.info("Motion 2.2")
        result = motion_client_node.move_to_pose(pick_pose,
                                                 cartesian_motion=True)
        if result.val != MoveItErrorCodes.SUCCESS:
            logger.error(f"Failed motion 2.2: {result.val}")
            motion_client_node.destroy_node()
            vision_client_node.destroy_node()
            demo_node.destroy_node()
            rclpy.shutdown()
            return 0

        # # 2.3 close the gripper
        logger.info("Motion 2.3")
        # if not REAL:
        #     motion_client_node.attach_object("dice", "tip")
        if REAL:
            reached_goal, stalled = motion_client_node.gripper_command(
                position=0.7)  # 0.0 = closed
            logger.info(
                f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})"
            )

        # 2.4 lift
        pick_pose.header.frame_id = "tip"
        pick_pose.header.stamp = demo_node.get_clock().now().to_msg()
        pick_pose.pose.position.x = 0.0
        pick_pose.pose.position.y = 0.0
        pick_pose.pose.position.z = -0.10
        pick_pose.pose.orientation.x = 0.0
        pick_pose.pose.orientation.y = 0.0
        pick_pose.pose.orientation.z = 0.0
        pick_pose.pose.orientation.w = 1.0
        logger.info("Motion 2.4")
        result = motion_client_node.move_to_pose(pick_pose,
                                                 cartesian_motion=True)
        if result.val != MoveItErrorCodes.SUCCESS:
            logger.error(f"Failed motion 2.4: {result.val}")
            motion_client_node.destroy_node()
            vision_client_node.destroy_node()
            demo_node.destroy_node()
            rclpy.shutdown()
            return 0

        # 3.1 Drop the dice
        pick_pose.header.frame_id = "checkerboard"
        pick_pose.header.stamp = demo_node.get_clock().now().to_msg()
        pick_pose.pose.position.x = 0.0
        pick_pose.pose.position.y = 0.0
        pick_pose.pose.position.z = -0.1
        pick_pose.pose.orientation.x = 0.0
        pick_pose.pose.orientation.y = 0.0
        pick_pose.pose.orientation.z = 0.0
        pick_pose.pose.orientation.w = 1.0

        logger.info("Motion 3.1")
        result = motion_client_node.move_to_pose(pick_pose,
                                                 cartesian_motion=True)
        if result.val != MoveItErrorCodes.SUCCESS:
            logger.error(f"Failed motion 3.1: {result.val}")
            motion_client_node.destroy_node()
            vision_client_node.destroy_node()
            demo_node.destroy_node()
            rclpy.shutdown()
            return 0

        # 3.2 open gripper
        if REAL:
            logger.info("Motion 3.2")
            reached_goal, stalled = motion_client_node.gripper_command(
                position=0.045)  # Open gripper


if __name__ == "__main__":
    main()
