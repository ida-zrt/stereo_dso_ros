#include "util/globalCalib.h"
#include "util/DatasetReader.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/ImmaturePoint.h"
#include <sensor_msgs/PointCloud2.h>
#include "IOWrapper/ROS/ROSOutputWrapper.h"

namespace dso
{
    namespace IOWrap
    {
        inline float avg(const float * x, int num)
        {
            float sum = 0;
            for(int i = 0 ; i < num; i ++)
            {
                sum += x[i];
            }
            sum /= num;
            return sum;
        }
        void addMP(const PointHessian *ph, std::vector<singlePoint> &mp_list, CalibHessian *HCalib, int status, float max_range)
        {
            float fxi = 1 / HCalib->fxl();
            float fyi = 1 / HCalib->fyl();
            float cxi = -HCalib->cxl() / HCalib->fxl();
            float cyi = -HCalib->cyl() / HCalib->fyl();
            float depth = 1.0f / ph->idepth_scaled;
            // point in camera coor
            Eigen::Vector3d mp_posi;
            mp_posi(0) = (ph->u * fxi + cxi) * depth;
            mp_posi(1) = (ph->v * fyi + cyi) * depth;
            mp_posi(2) = depth;
            if (mp_posi(2) > max_range)
                return;
            FrameHessian *host = ph->host;
            // point in world coor
            Eigen::Vector4d mp_posi_homo;
            mp_posi_homo.block(0, 0, 3, 1) = mp_posi;
            mp_posi_homo(3) = 1;
            mp_posi_homo = host->shell->camToWorld.matrix() * mp_posi_homo;
            mp_posi = mp_posi_homo.block(0, 0, 3, 1);
            singlePoint mp_posi2;
            mp_posi2.x = mp_posi(0);
            mp_posi2.y = mp_posi(1);
            mp_posi2.z = mp_posi(2);
            mp_posi2.I = avg(ph->color, MAX_RES_PER_POINT);
            mp_posi2.status = status;
            mp_list.push_back(mp_posi2);

            // // if the point is already in the list, don't add the point
            // for (int i = 0; i < mp_list.size(); i++)
            // {
            //     if ((mp_list[i] - mp_posi).norm() < 0.01)
            //         return;
            // }
        }

        void addMP(const ImmaturePoint *ph, std::vector<singlePoint> &mp_list, CalibHessian *HCalib)
        {
            float fxi = 1 / HCalib->fxl();
            float fyi = 1 / HCalib->fyl();
            float cxi = -HCalib->cxl() / HCalib->fxl();
            float cyi = -HCalib->cyl() / HCalib->fyl();
            float depth = 1.0f / ((ph->idepth_max + ph->idepth_min) * 0.5f);
            // point in camera coor
            Eigen::Vector3d mp_posi;
            mp_posi(0) = (ph->u * fxi + cxi) * depth;
            mp_posi(1) = (ph->v * fyi + cyi) * depth;
            mp_posi(2) = depth;

            FrameHessian *host = ph->host;
            // point in world coor
            Eigen::Vector4d mp_posi_homo;
            mp_posi_homo.block(0, 0, 3, 1) = mp_posi;
            mp_posi_homo(3) = 1;
            mp_posi_homo = host->shell->camToWorld.matrix() * mp_posi_homo;
            mp_posi = mp_posi_homo.block(0, 0, 3, 1);
            singlePoint mp_posi2;
            mp_posi2.x = mp_posi(0);
            mp_posi2.y = mp_posi(1);
            mp_posi2.z = mp_posi(2);
            mp_posi2.I = avg(ph->color, MAX_RES_PER_POINT);
            mp_posi2.status = 4;
            mp_list.push_back(mp_posi2);
        }

        KeyFramePoints extractKeyframePoints(const FrameHessian *KF, CalibHessian *HCalib)
        {
            KeyFramePoints kfp;
            int num_pts = KF->pointHessians.size() +
                          KF->pointHessiansMarginalized.size() +
                          KF->immaturePoints.size();
            ROS_INFO("Num of points in KF %d is %d\n",KF->frameID, num_pts);
            for (const auto ph : KF->pointHessians)
            {
                // printf("Adding active points to the map\n");
                addMP(ph, kfp, HCalib, 0);
            }

            for (const auto ph : KF->pointHessiansMarginalized)
            {
                // printf("Adding marginalized points to the map\n");
                addMP(ph, kfp, HCalib, 1);
            }

            // for (const auto ph : KF->pointHessiansOut)
            //     addMP(ph, kfp, HCalib, 2);

            // for (const auto ph : KF->potentialPointHessians)
            //     addMP(ph, kfp, HCalib, 3);

            for (const auto ph : KF->immaturePoints)
            {
                // printf("Adding immature points to the map\n");
                addMP(ph, kfp, HCalib);
            }

            return kfp;
        }
    }

}
