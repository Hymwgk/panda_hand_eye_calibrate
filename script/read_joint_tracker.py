#!/usr/bin/env python
# coding=UTF-8    
# 使用中文注释的话，就必须要添加上一条编码说明
# license removed for brevity
import rospy
#引入系统模块，可用于读取外部参数
import os
import sys   
import time
#ros tf相关模块
import roslib
import math
import tf
#使用机械臂/joint_states话题，读取其中的各个关节弧度角
from sensor_msgs.msg  import JointState
#键盘检测模块
import tty
import termios

#操作yaml文件模块
#import yaml     #读
from ruamel import yaml    #写

#moveit模块
import moveit_commander
import moveit_msgs.msg
# tf模块
from tf import TransformListener
#numpy
import numpy as np


def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)




#多行注释
def joint_reader():

    return 
def marker_reader():

    return


#让机械臂从当前的位置移动到指定的目标位置处
def move2goal():

    return



#读取yaml文件数据
def read_yaml(yaml_file):
    print("***读取yaml文件***")
    #file = open(yaml_file,'r')
    with open(yaml_file,'r') as file:
        #file_data=file.read()
        
        #print type(file_data)
        #print(yaml.load(file.read(),Loader=yaml.Loader))
        file_data = yaml.load(file.read(),Loader=yaml.Loader)
        #print file_data
        file.close()
    return file_data

#获取当前机械臂各关节角度
def get_joint_angle():
    #单次订阅关节消息，存在data中
    data=rospy.wait_for_message("/joint_states", JointState, timeout=None)
    #将前七个关节角度保存为元组
    arm_angles=[data.position[0],data.position[1],data.position[2],data.position[3],data.position[4],data.position[5],data.position[6]]
    #返回该元组
    return arm_angles

def write_yaml(yaml_file,data):
    print"***写文件***"
    with open(yaml_file, 'w') as f:
        yaml.dump(data,f,Dumper=yaml.RoundTripDumper)

    return



#并不是直接运行这个代码就行了，在这个代码之前，还需要提前加载好urdf等等东西
class Moveit_Demo:
    def __init__(self,angles):
        #初始化tf监听器
        self.tf = TransformListener()

        #初始化    括号中必须加参数么？这个参数如何设定？
        moveit_commander.roscpp_initialize(sys.argv)

        #初始化movegroup
        arm = moveit_commander.MoveGroupCommander("panda_arm")
        #显示轨迹，使用rviz可监视
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        #设置机械臂允许的误差值，默认就好
        arm.set_goal_joint_tolerance(0.001)

        #设置需要读取的两个变换关系
        #marker2camera={}
        #ee2base={}
        
        # create empty matrices to save tracked data
        ee2base = np.empty(shape=[0,3])     #创建一个行数不定，列数为3的矩阵
        marker2camera = np.empty(shape=[0,3])


        #一直等待接收到标签和相机之间的转换关系
        rospy.loginfo("Wating for Kinect and marker to came up")
        get_transform=False
        while not get_transform:
            try:
                #尝试查看机器人基座base与桌面标签之间的转换
                trans, _ = self.tf.lookupTransform('/ar_marker_7', '/kinect2_link', rospy.Time(0))
                get_transform = True
                rospy.loginfo("got transform complete")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        #顺序移动到之前的示教位置
        print("Got {} arm states".format(len(angles)))
        for i in range(0,len(angles)):
            position_i = 'angles_'+ str(i)
            
            #开始移动
            print ("Moving to "+position_i)
            arm.go(angles[position_i],wait=True)
            arm.stop()
            #暂停一下，让机械臂稳定下来不乱晃
            time.sleep(2)
            
            #读取当前pose下标定板二维码与相机间的tf变换关系
            #marker2camera_i = 'marker_'+ str(i)       #想要使用str()必须保证  str不被自定义
            #marker2camera[marker2camera_i]=self.get_tf("ar_marker_7","kinect2_rgb_optical_frame")
            marker2camera_i=self.lookupTransformMatrix(self.tf,"ar_marker_7","kinect2_link")
            trans_camera, rot, rot_euler = self.getTfFromMatrix(np.linalg.inv(marker2camera_i)) #提取数据
            
            #读取当前pose下末端执行器与基座之间的变换关系
            #ee2base_i='ee2base_' + str(i)
            #ee2base[ee2base_i]=self.get_tf("panda_EE","panda_link0")
            ee2base_i=self.lookupTransformMatrix(self.tf,"panda_EE","panda_link0")
            trans_panda, rot, rot_euler = self.getTfFromMatrix(np.linalg.inv(ee2base_i)) #提取数据

            #写入np矩阵
            marker2camera= np.vstack([marker2camera,np.asarray(trans_camera)])
            ee2base = np.vstack([ee2base,np.asarray(trans_panda)])




        #最终回到home位置
        print("Moving back to angles_0")
        arm.go(angles['angles_0'],wait=True)
        
        #保存之前记录的相机与二维码之间的变换关系，写到yaml文件中
        #self.write_yaml(os.environ['HOME']+"/catkin_ws/src/panda_hand_eye_calibrate/config/marker2camera.yaml",marker2camera)
        self.saveData(ee2base,marker2camera)
        
        #保存之前记录的末端执行器与基座之间的变换关系，写到yaml文件中
        #self.write_yaml(os.environ['HOME']+"/catkin_ws/src/panda_hand_eye_calibrate/config/ee2base.yaml",ee2base)
            
        #关闭并退出
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    #获取某两个坐标系间的变换关系
    def get_tf(self,child_frame,parent_frame):
        #data=rospy.wait_for_message("/joint_states", JointState, timeout=None)
        pq=[]
        try:
            position, quaternion = self.tf.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            print(position+quaternion)
            pq=position+quaternion
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("丢失标签！标定无效！")

        return pq

    #写入yaml文件数据
    def write_yaml(self,yaml_file,data):
        print("***写文件***")
        with open(yaml_file, 'w') as f:
            yaml.dump(data,f,Dumper=yaml.RoundTripDumper)
        return

    #保存相机和机械臂的各个位置点，以numpy数据形式保存
    def saveData(self,ee2base,marker2camera):
        # write to yaml file
        print 'write tracking data to files'
        savePath=os.environ['HOME']+"/catkin_ws/src/panda_hand_eye_calibrate/config/"
        np.savetxt('%see2base' % (savePath), ee2base, delimiter=',', fmt='%.4f')
        np.savetxt('%smarker2camera' % (savePath), marker2camera, delimiter=',', fmt='%.4f')
        return

    # 从4*4的矩阵中获取出详细的资料，返回值都是numpy矩阵
    def getTfFromMatrix(self,matrix):
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
        return trans, tf.transformations.quaternion_from_euler(*angles), angles

    # 查询两个坐标系之间的TF关系，返回值是numpy4*4矩阵
    def lookupTransformMatrix(self,tf_listener, parent, child):
        #tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))
        try:
            trans, rot = tf_listener.lookupTransform(parent, child, rospy.Time(0))  #注意这里返回的两个值都是列表形式
            euler = tf.transformations.euler_from_quaternion(rot)
            source_target = tf.transformations.compose_matrix(translate = trans, angles = euler)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("丢失标签！标定无效！")
            #source_target=False
        return source_target








if __name__ == '__main__':
    function = sys.argv[1]
    print(function)       #测试一下读取的参数

    #首先要初始化这个节点，名为read_joint_marker
    rospy.init_node('read_joint_marker',anonymous=True)     


    #joint_listener = tf.TransformListener()  
    data_angles={}

    if function == "record":   #如果输入的参数是record，就执行记录
        print "按下s保存当前关节角度；输入d完成保存；按下q结束节点；按下p转到标定；机械臂第一个位置将会被设置为home位置"
        count =0
        while True:
            key=readkey()
           
            if key=='s': #如果按下按键s  就代表save当前的关节角度
                print("保存当前的关节角")
                #为当前关节状态设置key          
                position_id = 'angles_'+ str(count)       #想要使用str()必须保证  str不被自定义
                count+=1
                #为当前key设置值，构建一个字典
                data_angles[position_id]=get_joint_angle()
                #break  不退出，要保存多个姿态



            elif key=='d':   #如果按下d  就代表位置记录完成，退出记录
                #把之前保存的字典写入yaml
                write_yaml(os.environ['HOME'] +"/catkin_ws/src/panda_hand_eye_calibrate/config/joint_angle.yaml",data_angles)

                print"位置记录完成，退出记录，是否进行二维码定位？按下y/n"
                while True:
                    key1=readkey()
                    if key1=='y': 
                        print"远离机械臂，5秒后进行二维码定位..."
                        key = 'p'
                        time.sleep(5) 
                        #执行标定代码
                        break

                    elif key1=='n':
                        print "退出记录"
                        key='q'    #使得后续退出记录
                        #做一些后续处理
                        break
                    
                    else: #如果没有按下，就一直循环等待着
                        pass


            if key=='q':
                print"结束节点"
                break

            if key=='p':
                #print"准备标定"
                function="move"
                break


            else:     #如果没按下指定的按键就跳过，继续循环
                pass

    if function=="move":  #不执行记录，只检测二维码位置
        print"开始二维码定位"
        joint_angles = read_yaml(os.environ['HOME'] +"/catkin_ws/src/panda_hand_eye_calibrate/config/joint_angle.yaml")
        print("已读取共{}个机械臂姿态".format(len(joint_angles)))
        move=Moveit_Demo(joint_angles)
        

        


rospy.spin()      #和roscpp中不同，这句话单纯是让订阅在当前节点终止的时候   停止订阅，