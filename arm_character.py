#!/usr/bin/env python
# -*- coding: utf-8 -*-    #日本語のコメントを入れるためのおまじない

import sys, math, copy
import rospy, tf, geometry_msgs.msg

from moveit_commander import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, PoseStamped

# グリッパを閉じる
def open_gripper():
    print("Opening Gripper...")
    gripper_joint_angle[0] = -0.01
    gripper_group.set_joint_value_target(gripper_joint_angle)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)

# グリッパを開く
def close_gripper():
    print("Closing Gripper...")
    gripper_joint_angle[0] = 0.006
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
    
    contact_pos_x = 0.2171559147703515
    contact_pos_y = 0.011025279431936829
    contact_pos_z = 0.04210407588819228
    contact_orient_x = -0.00026762818390114004
    contact_orient_y = 0.009967115970258395
    contact_orient_z = 0.02684010706420044
    contact_orient_w = 0.9995900127688215
    pick_up_pos_z = contact_pos_z + 0.05
    
    
    # グリッパの初期値を取得
    gripper_joint_angle = gripper_group.get_current_joint_values()
    print("Get Current Gripper angle:\n{}\n".format(gripper_joint_angle))
    
    # グリッパを開く
    open_gripper()    

    
    # init
    rospy.loginfo( "Init Pose")
    pose_target = Pose()
    pose_target.position.x =  0.12594757969107834
    pose_target.position.y =  0.06501309219622559
    pose_target.position.z =  0.2436765529398073
    pose_target.orientation.x = 0.0011796973119297986 
    pose_target.orientation.y =  -0.004448148067920811
    pose_target.orientation.z = 0.25634598727499985  
    pose_target.orientation.w =  0.9665741343016705  
    group.set_joint_value_target(pose_target,True)
    group.go()    
    rospy.sleep(0.2)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )      
    
    
    # Pick
    rospy.loginfo( "Pick Pose")
    pose_target.position.x =  0.2984740895418314
    pose_target.position.y =  0.18264144141042132
    pose_target.position.z =  0.0541949815927592
    pose_target.orientation.x = -0.0019327509684683615 
    pose_target.orientation.y =  0.006626763674297406  
    pose_target.orientation.z =  0.2799859723362082  
    pose_target.orientation.w =  0.9599792736157625  
    group.set_joint_value_target(pose_target,True)
    group.go()    
    rospy.sleep(0.1) # 0.1秒のスリープ
    close_gripper()
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) ) 
    
    # Pick Up
    rospy.loginfo( "Pick Up Pose")    
    pose_target.position.z =  pose_target.position.z + 0.1
    group.set_joint_value_target(pose_target,True)
    group.go()    
    rospy.sleep(0.2)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) ) 
    
    
    # Move to over first edge
    rospy.loginfo( "over first edge")
    pose_target.position.x =  0.18774697908944238
    pose_target.position.y =  0.11357009086053639
    pose_target.position.z =  0.07646480862737073
    pose_target.orientation.x  = -0.00021700812418394853
    pose_target.orientation.y =  -0.00021700812418394853
    pose_target.orientation.z =  0.2829364956739366
    pose_target.orientation.w =  0.9591383378583592
    group.set_joint_value_target(pose_target,True)
    group.go()    
    rospy.sleep(0.1) # 0.1秒のスリープ
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )    

    # first edge
    rospy.loginfo( "first edge")
    pose_target.position.x =  0.18838382140350882
    pose_target.position.y =  0.11398162703513859
    pose_target.position.z =  0.043213407777021075
    pose_target.orientation.x  = 0.0021700643942005097
    pose_target.orientation.y =  -0.007356392646135441
    pose_target.orientation.z =  0.28292825680249484
    pose_target.orientation.w =   0.9591104085612159
    group.set_joint_value_target(pose_target,True)
    group.go()    
    rospy.sleep(0.1) # 0.1秒のスリープ
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )      
    
    # second edge
    rospy.loginfo( "second edge")
    pose_target.position.x =  0.24536064147045683
    pose_target.position.y =  0.033884418566354005
    pose_target.position.z =  0.039413064345058306
    pose_target.orientation.x  = 0.002154424046490024
    pose_target.orientation.y = -0.029830462666504718
    pose_target.orientation.z = 0.07200243188762416
    pose_target.orientation.w = 0.9969559427358855
    group.set_joint_value_target(pose_target,True)
    group.go()    
    rospy.sleep(0.1) # 0.1秒のスリープ
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )  
    
    rospy.sleep(0.1)
    pose_current = group.get_current_pose()
    rospy.loginfo( "Get Current Pose:\n{}\n".format( pose_current ) )
