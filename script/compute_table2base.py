#!/usr/bin/env python
#coding=utf-8
#适用于panda机器人：     读取base与桌面上marker之间的坐标系变换关系，保存成一个yaml文件记录下来
#在此之前需要进行如下工作：
#提前运行ar_marker识别出桌面标签
#提前发布kinect与base之间的静态变换关系（即手眼标定）
#

import tf
import rospy
import numpy as np
import yaml
import os



def main():
    # initialize ros node
    rospy.init_node('compute_table2base')

    tf_listener=tf.TransformListener()
    get_transform=False
    #等待接收正确的机器人与相机之间的关系
    while not get_transform:
        try:
            #尝试查看机器人基座base与kinect之间的转换
            trans, _ = tf_listener.lookupTransform('/kinect2_link', '/panda_link0', rospy.Time(0))
            trans, _ = tf_listener.lookupTransform('/ar_marker_6', '/panda_link0', rospy.Time(0))

            get_transform = True
            rospy.loginfo("Got transform from kinect  to base and table_marker to  base ")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    #def lookupTransform(self, target_frame, source_frame, time):
    #注意这里，target_frame和source_frame指的是坐标的变换，而不是坐标系的变换
    #此时返回的变换是从target_frame坐标系变换到source_frame坐标系，
    #所以，target_frame是父坐标系，source_frame才是子坐标系
    #因此，这里parent: =/ar_marker_6     child: =/panda_link0
    trans, rot = tf_listener.lookupTransform('/ar_marker_6', '/panda_link0', rospy.Time(0))
    euler = tf.transformations.euler_from_quaternion(rot)

    # 获取存放变换文件的地址
    filesPath = rospy.get_param('~files_path')

    # save results to yaml file  写base与桌面标签之间的关系
    f = open('%stable2base.yaml' % (filesPath), 'w')
    lines = ['trans: [', 'rot: [', 'rot_euler: [']

    for elem in trans: lines[0] += str(elem) + ', '  #trans
    lines[0] += ']\n'
    for elem in rot: lines[1] += str(elem) + ', '  #rot
    lines[1] += ']\n'
    for elem in euler: lines[2] += str(elem) + ', '  #rot_euler
    lines[2] += ']\n'

    lines.append('parent: /ar_marker_6\n')
    lines.append('child: /panda_link0\n')

    f.writelines(lines)
    f.close()

    print 'Done !'


if __name__=='__main__':
    main()




