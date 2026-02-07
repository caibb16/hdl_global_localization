#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动初始位姿计算节点
用于调用 hdl_global_localization 服务计算初始位姿，并发布到 /initialpose 话题
"""

import rospy
import tf
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from hdl_global_localization.srv import (
    SetGlobalLocalizationEngine,
    SetGlobalMap, 
    QueryGlobalLocalization
)
from std_srvs.srv import Trigger, TriggerResponse


class AutoInitialPose:
    def __init__(self):
        # 这里的false表示节点名称不允许重复  不要带一串数字
        rospy.init_node('auto_initialpose', anonymous=False)
        
        # 参数配置
        self.engine_name = rospy.get_param('~engine', 'FPFH_RANSAC')  # BBS, FPFH_RANSAC, FPFH_TEASER
        self.map_topic = rospy.get_param('~map_topic', '/globalmap')
        # 这里launch中传入/livox/pointcloud2
        self.cloud_topic = rospy.get_param('~cloud_topic', '/velodyne_points')
        self.max_candidates = rospy.get_param('~max_candidates', 1)
        self.auto_mode = rospy.get_param('~auto_mode', False)  # True: 自动持续定位, False: 手动触发
        self.localization_rate = rospy.get_param('~localization_rate', 1.0)  # 自动模式下的频率(Hz)
        
        # 服务客户端
        self.set_engine_client = None
        self.set_map_client = None
        self.query_client = None
        
        # 发布器  定义的两个发布话题
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        # 发布状态信息
        self.status_pub = rospy.Publisher('~status', String, queue_size=1)
        
        # 订阅器
        self.cloud_sub = None
        self.map_received = False
        self.latest_cloud = None
        
        rospy.loginfo("正在等待 hdl_global_localization 服务...")
        self.init_services()
        
        # 订阅地图
        rospy.Subscriber(self.map_topic, PointCloud2, self.map_callback, queue_size=1)
        
        # 订阅点云
        if self.auto_mode:
            rospy.loginfo("启动自动模式，频率: {} Hz".format(self.localization_rate))
            self.cloud_sub = rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)
            
        # 手动触发服务  这里换成了Trigger类型
        self.trigger_service = rospy.Service('~trigger', Trigger, self.trigger_callback)

        
        rospy.loginfo("自动初始位姿节点已启动")
        rospy.loginfo("算法引擎: {}".format(self.engine_name))
        rospy.loginfo("地图话题: {}".format(self.map_topic))
        rospy.loginfo("点云话题: {}".format(self.cloud_topic))
        
    def init_services(self):
        """初始化服务客户端"""
        try:
            #等待hdl节点的这些服务启动
            rospy.wait_for_service('/hdl_global_localization/set_engine', timeout=5)
            rospy.wait_for_service('/hdl_global_localization/set_global_map', timeout=5)
            rospy.wait_for_service('/hdl_global_localization/query', timeout=5)
            

            #创建服务的客户端代理句柄，方便调用这些服务
            self.set_engine_client = rospy.ServiceProxy(
                '/hdl_global_localization/set_engine', 
                SetGlobalLocalizationEngine
            )
            self.set_map_client = rospy.ServiceProxy(
                '/hdl_global_localization/set_global_map',
                SetGlobalMap
            )
            self.query_client = rospy.ServiceProxy(
                '/hdl_global_localization/query',
                QueryGlobalLocalization
            )
            
            # 设置算法引擎
            # 使用句柄，调用服务设置引擎，，这个不需要回调函数
            self.set_engine_client(engine_name=String(data=self.engine_name))
            rospy.loginfo("算法引擎设置为: {}".format(self.engine_name))
            
        except rospy.ROSException as e:
            rospy.logerr("服务初始化失败: {}".format(e))
            rospy.signal_shutdown("服务不可用")
            
    def map_callback(self, msg):
        """接收全局地图并设置"""
        if self.map_received:
            return
            
        rospy.loginfo("接收到全局地图，正在设置...")
        try:

            # 这里真正的浪费时间是在hdl_global_localization节点中设置地图
            self.set_map_client(global_map=msg)
            self.map_received = True
            rospy.loginfo("全局地图设置成功")
            self.publish_status("地图已加载")
        except rospy.ServiceException as e:
            rospy.logerr("设置地图失败: {}".format(e))
            
    def cloud_callback(self, msg):
        """接收点云数据"""
        self.latest_cloud = msg
        
    # def trigger_callback(self, req):
    #     """手动触发定位服务"""
    #     rospy.loginfo("收到手动触发请求")
    #     if self.latest_cloud is not None:
    #         self.compute_and_publish_pose(self.latest_cloud)
    #     else:
    #         # 等待一个点云
    #         rospy.loginfo("等待点云数据...")
    #         cloud_msg = rospy.wait_for_message(self.cloud_topic, PointCloud2, timeout=5)
    #         self.compute_and_publish_pose(cloud_msg)
    #     return []
    def trigger_callback(self, req):
        rospy.loginfo("收到手动触发请求")

        if not self.map_received:
            msg = "地图尚未加载，无法触发定位"
            rospy.logwarn(msg)
            return TriggerResponse(success=False, message=msg)

        try:
            if self.latest_cloud is not None:
                ok = self.compute_and_publish_pose(self.latest_cloud)
            else:
                rospy.loginfo("等待点云数据...")
                cloud_msg = rospy.wait_for_message(self.cloud_topic, PointCloud2, timeout=5)
                ok = self.compute_and_publish_pose(cloud_msg)

            if ok:
                return TriggerResponse(success=True, message="定位成功，已发布 /initialpose")
            else:
                return TriggerResponse(success=False, message="定位失败（无有效候选或服务异常）")

        except rospy.ROSException as e:
            return TriggerResponse(success=False, message="等待点云超时/异常: {}".format(e))
            
    def compute_and_publish_pose(self, cloud_msg):
        """计算并发布初始位姿
        Returns:
            bool: True=定位成功并已发布 /initialpose；False=失败或跳过
        """
        if not self.map_received:
            rospy.logwarn("地图尚未加载，跳过定位")
            return False

        try:
            rospy.loginfo("开始计算初始位姿...")

            # 调用全局定位服务
            response = self.query_client(
                max_num_candidates=self.max_candidates,
                cloud=cloud_msg
            )

            if len(response.poses) == 0:
                rospy.logwarn("未找到有效的定位结果")
                self.publish_status("定位失败")
                return False

            # 获取最佳位姿
            best_pose = response.poses[0]
            inlier_fraction = response.inlier_fractions[0] if response.inlier_fractions else 0.0
            error = response.errors[0] if response.errors else 0.0

            rospy.loginfo("定位成功! 内点比率: {:.2f}, 误差: {:.4f}".format(inlier_fraction, error))

            # 构造并发布 initialpose 消息
            initialpose_msg = PoseWithCovarianceStamped()
            initialpose_msg.header.stamp = rospy.Time.now()
            initialpose_msg.header.frame_id = "map"

            initialpose_msg.pose.pose.position = best_pose.position
            initialpose_msg.pose.pose.orientation = best_pose.orientation

            # 设置协方差（可以根据误差调整）
            initialpose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

            self.initialpose_pub.publish(initialpose_msg)

            # 转换四元数为欧拉角并打印
            quaternion = (
                best_pose.orientation.x,
                best_pose.orientation.y,
                best_pose.orientation.z,
                best_pose.orientation.w
            )
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

            rospy.loginfo("位姿: x={:.2f}, y={:.2f}, z={:.2f}".format(
                best_pose.position.x, best_pose.position.y, best_pose.position.z
            ))
            rospy.loginfo("姿态: roll={:.2f}°, pitch={:.2f}°, yaw={:.2f}°".format(
                roll*180/3.14159, pitch*180/3.14159, yaw*180/3.14159
            ))

            self.publish_status("定位成功")
            return True

        except rospy.ServiceException as e:
            rospy.logerr("定位服务调用失败: {}".format(e))
            self.publish_status("服务异常")
            return False
       
    def publish_status(self, status):
        """发布状态信息"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        
    def run(self):
        """主循环"""
        if self.auto_mode and self.map_received:
            rate = rospy.Rate(self.localization_rate)
            while not rospy.is_shutdown():
                if self.latest_cloud is not None:
                    self.compute_and_publish_pose(self.latest_cloud)
                rate.sleep()
        else:
            rospy.spin()


if __name__ == '__main__':
    try:
        node = AutoInitialPose()
        node.run()
    except rospy.ROSInterruptException:
        pass