#!/usr/bin/env python
# -*- coding: utf-8 -*-    #日本語のコメントを入れるためのおまじない

import sys, math, copy
import rospy, tf, geometry_msgs.msg

from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, PoseStamped

# グリッパを閉じる
def close_gripper():
    print("Closing Gripper...")
    gripper_joint_angle[0] = -0.01
    gripper_group.set_joint_value_target(gripper_joint_angle)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)

# グリッパを開く
def open_gripper():
    print("Opening Gripper...")
    gripper_joint_angle[0] = 0 
    gripper_group.set_joint_value_target(gripper_joint_angle)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)
    
def set_position(pose_target, x, y, z):
    pose_target.position.x
    pose_target.position.y
    pose_target.position.x
 
def set_orientation(pose_target, x, y, z, w):
    pose_target.orientation.x = x
    pose_target.orientation.y = y
    pose_target.orientation.z = z
    pose_target.orientation.w = w

if __name__ == '__main__':
    
    node_name = "arm_character"
    rospy.init_node(node_name, anonymous=True ) # ノードの初期化

    ## MoveGroupCommanderオブジェクトのインスタンス生成。これがジョイントへのインタフェースになる。
    ## このインタフェースは運動計画と動作実行に使われる。
    group = MoveGroupCommander("arm")
    gripper_group = MoveGroupCommander("gripper") # 追加
    
    group.set_planning_time(100.0) # 動作計画に使う時間[s]の設定
    
    # 初期姿勢の取得
    pose_init = group.get_current_pose()  # エンドエフェクタの位置(x, y, z)を取得
    rospy.loginfo( "Get Initial Pose\n{}".format( pose_init ) )
    rpy_init  = group.get_current_rpy()   # エンドエフェクタの姿勢(roll, pitch, yaw)を取得
    rospy.loginfo( "Get Initial RPY:{}".format( rpy_init ) )
    
    # グリッパの初期値を取得
    gripper_joint_angle = gripper_group.get_current_joint_values()
    print("Get Current Gripper angle:\n{}\n".format(gripper_joint_angle))    

    open_gripper()
    
    rospy.sleep(0.1)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
