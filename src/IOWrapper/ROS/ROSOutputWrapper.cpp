#include "util/settings.h"
#include "util/globalCalib.h"
#include "util/DatasetReader.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/ImmaturePoint.h"
#include "IOWrapper/ROS/ROSOutputWrapper.h"
#include "IOWrapper/Output3DWrapper.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <rosbag/bag.h>

namespace dso
{
    namespace IOWrap
    {
        // ===============初始化==============================
        ROSOutputWrapper::ROSOutputWrapper(
            const char *pc_topic,
            const char *pos_topic,
            const char *traj_topic,
            const char *frame_id,
            float lw, ros::NodeHandle &nh)
        {
            frameID = frame_id;
            // setup point cloud map
            pc_publisher = nh.advertise<sensor_msgs::PointCloud2>(pc_topic, 10);
            pc_output.header.frame_id = frameID;
            ROS_INFO("publishing point cloud map to topic: %s", pc_topic);

            // setup trajectory
            traj_publisher = nh.advertise<visualization_msgs::Marker>(traj_topic, 10);
            trajectory_marker.id = 0;
            trajectory_marker.header.frame_id = frameID;
            trajectory_marker.ns = "cam_traj";
            trajectory_marker.action = visualization_msgs::Marker::ADD;
            trajectory_marker.pose.orientation.w = 1.0;
            trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
            trajectory_marker.scale.x = lw;
            trajectory_marker.color.r = 1.0;
            trajectory_marker.color.a = 1.0;

            // setup pose publisher
            pos_publisher = nh.advertise<geometry_msgs::PoseStamped>(pos_topic, 10);
            camCurrentPose.header.frame_id = frameID;
        }
        ROSOutputWrapper::~ROSOutputWrapper() {}

        // ==================发布地图点,和轨迹信息======================
        void ROSOutputWrapper::publishKeyframes(std::vector<FrameHessian *> &frames, bool final, CalibHessian *HCalib)
        {
            for (const auto &fh : frames)
            {
                if (map_points_with_KID.find(fh->frameID) == map_points_with_KID.end())
                {
                    printf("%d is a new frame\n", fh->frameID);
                }
                // ?? 如何减少计算量
                // 用关键帧维护每个点, 关键帧的个数是由滑动窗口的大小决定的

                // 将每个关键帧上的地图点保存起来
                map_points_with_KID[fh->frameID] = extractKeyframePoints(fh, HCalib);

                // 保存每个关键帧上的轨迹点
                geometry_msgs::Point p;
                Eigen::Matrix<double, 3, 1> T = fh->shell->camToWorld.translation().transpose();
                p.x = T(0);
                p.y = T(1);
                p.z = T(2);
                camTraj[fh->frameID] = p;

                // 保存每个关键帧的相机姿态
                Eigen::Vector3d position = fh->shell->camToWorld.translation().transpose();
                geometry_msgs::Pose camPose;

                camPose.position.x = position(0);
                camPose.position.y = position(1);
                camPose.position.z = position(2);

                camPose.orientation.x = fh->shell->camToWorld.so3().unit_quaternion().x();
                camPose.orientation.y = fh->shell->camToWorld.so3().unit_quaternion().y();
                camPose.orientation.z = fh->shell->camToWorld.so3().unit_quaternion().z();
                camPose.orientation.w = fh->shell->camToWorld.so3().unit_quaternion().w();

                allCamPose[fh->frameID] = camPose;
            }
            // 转换点云并发布地图
            map2cloud();

            // 提取每个关键帧的位置,转换至traj marker, 并发布
            traj_publish(frames);

            // 保存相机位姿， 参考以下原始的 printResults
            if (final)
            {
                // printf("saveing results to ~/results.bag");
                // saveResults("~/results.bag");
            }
        }

        // =================发布相机位姿=======================
        void ROSOutputWrapper::publishCamPose(FrameShell *frame, CalibHessian *HCalib)
        {
            Eigen::Vector3d position = frame->camToWorld.translation().transpose();

            camCurrentPose.pose.position.x = position(0);
            camCurrentPose.pose.position.y = position(1);
            camCurrentPose.pose.position.z = position(2);

            camCurrentPose.pose.orientation.x = frame->camToWorld.so3().unit_quaternion().x();
            camCurrentPose.pose.orientation.y = frame->camToWorld.so3().unit_quaternion().y();
            camCurrentPose.pose.orientation.z = frame->camToWorld.so3().unit_quaternion().z();
            camCurrentPose.pose.orientation.w = frame->camToWorld.so3().unit_quaternion().w();

            camCurrentPose.header.stamp = ros::Time::now();
            pos_publisher.publish(camCurrentPose);
        }

        // =======================点云转换为ros格式并发布==================
        void ROSOutputWrapper::map2cloud()
        {
            // pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            int mapsize = 0;
            for (auto &kfp : map_points_with_KID)
            {
                mapsize += kfp.second.size();
            }

            cloud.width = mapsize;
            cloud.height = 1;
            cloud.points.resize(cloud.width * cloud.height);
            int p = 0;

            for (auto &kfp : map_points_with_KID)
            {
                for (auto &points : kfp.second)
                {
                    cloud.points[p].x = points.x;
                    cloud.points[p].y = points.y;
                    cloud.points[p].z = points.z;
                    // cloud.points[p].intensity = points.I;
                    // 这里需要点云的颜色为图像上的rgb（灰度）值
                    cloud.points[p].r = points.I;
                    cloud.points[p].g = points.I;
                    cloud.points[p].b = points.I;
                    p += 1;
                }
            }

            pcl::toROSMsg(cloud, pc_output);
            pc_output.header.frame_id = frameID;
            pc_output.header.stamp = ros::Time::now();
            pc_publisher.publish(pc_output);
            ROS_INFO("point cloud map published");
        }

        // =======================发布轨迹==========================
        void ROSOutputWrapper::traj_publish(std::vector<FrameHessian *> &frames)
        {
            trajectory_marker.points.clear();
            for (auto &point_with_id : camTraj)
            {
                trajectory_marker.points.push_back(point_with_id.second);
            }
            trajectory_marker.header.stamp = ros::Time::now();
            traj_publisher.publish(trajectory_marker);
        }

        // TODO!!: 如何确定保存的是对的？
        void ROSOutputWrapper::saveResults(const char *path)
        {
            rosbag::Bag slam_bag;
            slam_bag.open(path, rosbag::bagmode::Write);

            slam_bag.write("/trajectory_result", ros::Time::now(), trajectory_marker);
            slam_bag.write("/map_result", ros::Time::now(), pc_output);
            ros::Time poseStart = ros::Time::now();
            int i = 0;
            for (auto &camPose : allCamPose)
            {
                slam_bag.write("/cam_pose_list", poseStart + ros::Duration(0.1 * i), camPose.second);
            }
            slam_bag.close();
        }
    }
}