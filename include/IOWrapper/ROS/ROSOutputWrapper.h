#pragma once
#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

namespace dso
{

    class FrameHessian;
    class CalibHessian;
    class FrameShell;

    namespace IOWrap
    {
        struct singlePoint
        {
            double x;
            double y;
            double z;
            double I;
            int status;
        };
        typedef std::vector<singlePoint> KeyFramePoints;
        // typedef std::vector<singlePoint> CamTraj;

        void addMP(const PointHessian *ph, std::vector<singlePoint> &mp_list, CalibHessian *HCalib, int status, float max_range = 200);
        void addMP(const ImmaturePoint *ph, std::vector<singlePoint> &mp_list, CalibHessian *HCalib);
        KeyFramePoints extractKeyframePoints(const FrameHessian *KF, CalibHessian *HCalib);

        class ROSOutputWrapper : public Output3DWrapper
        {
        public:
            ROSOutputWrapper(const char *pc_topic, const char *pos_topic, const char *traj_topic, const char *frame_id, float lw, ros::NodeHandle &nh);
            ~ROSOutputWrapper();
            // ===============这些是 output3dwrapper 基类 中的虚函数， 相关的函数需要实现==============
            // void publishGraph(const std::map<long, Eigen::Vector2i> &connectivity);
            void publishKeyframes(std::vector<FrameHessian *> &frames, bool final, CalibHessian *HCalib);
            void publishCamPose(FrameShell *frame, CalibHessian *HCalib);
            // void pushLiveFrame(FrameHessian *image);
            // void pushDepthImage(MinimalImageB3 *image);
            // bool needPushDepthImage();
            // void pushDepthImageFloat(MinimalImageF *image, FrameHessian *KF);

            // ============ 这里也可以保存以下结果 ================
            void saveResults(const char* path);

        private:
            // map
            ros::Publisher pc_publisher;
            std::map<int, KeyFramePoints> map_points_with_KID;
            std::map<int, geometry_msgs::Point> camTraj;
            sensor_msgs::PointCloud2 pc_output;
            void map2cloud(); // publish point cloud map

            // traj
            ros::Publisher traj_publisher;
            visualization_msgs::Marker trajectory_marker;
            void traj_publish(std::vector<FrameHessian *> &frames);

            // cam pose
            ros::Publisher pos_publisher;
            geometry_msgs::PoseStamped camCurrentPose;
            std::map<int, geometry_msgs::Pose> allCamPose;
            void camPosePublish(FrameHessian *frame);

            // other
            const char *frameID;
        };
    }
}
