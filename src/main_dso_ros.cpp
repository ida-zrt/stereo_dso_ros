/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/ROS/ROSOutputWrapper.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "util/DatasetReader.h"

std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
bool useSampleOutput = false;
bool preload = false;
int mode = 0;
float lw = 0.1;

float playbackSpeed = 0; // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.

using namespace dso;

void settingsDefault(int preset)
{
    printf("\n=============== PRESET Settings: ===============\n");
    if (preset == 0 || preset == 1)
    {
        printf("DEFAULT settings:\n"
               "- %s real-time enforcing\n"
               "- 2000 active points\n"
               "- 5-7 active frames\n"
               "- 1-6 LM iteration each KF\n"
               "- original image resolution\n",
               preset == 0 ? "no " : "1x");

        playbackSpeed = (preset == 0 ? 0 : 1);
        preload = preset == 1;

        setting_desiredImmatureDensity = 1500; //original 1500. set higher
        setting_desiredPointDensity = 2000;    //original 2000
        setting_minFrames = 5;
        setting_maxFrames = 7;
        setting_maxOptIterations = 6;
        setting_minOptIterations = 1;

        setting_kfGlobalWeight = 0.3f;                  // original is 1.0. 0.3 is a balance between speed and accuracy. if tracking lost, set this para higher
        setting_maxShiftWeightT = 0.04f * (640 + 360);  // original is 0.04f * (640+480); this para is depend on the crop size.
        setting_maxShiftWeightR = 0.04f * (640 + 360);  // original is 0.0f * (640+480);
        setting_maxShiftWeightRT = 0.02f * (640 + 360); // original is 0.02f * (640+480);*/

        setting_logStuff = false;
    }
    else if (preset == 2 || preset == 3)
    {
        printf("FAST settings:\n"
               "- %s real-time enforcing\n"
               "- 800 active points\n"
               "- 4-6 active frames\n"
               "- 1-4 LM iteration each KF\n"
               "- 424 x 320 image resolution\n",
               preset == 0 ? "no " : "5x");

        playbackSpeed = (preset == 2 ? 0 : 5);
        preload = preset == 3;
        setting_desiredImmatureDensity = 600;
        setting_desiredPointDensity = 800;
        setting_minFrames = 4;
        setting_maxFrames = 6;
        setting_maxOptIterations = 4;
        setting_minOptIterations = 1;

        benchmarkSetting_width = 424;
        benchmarkSetting_height = 320;

        setting_logStuff = false;
    }

    printf("==============================================\n");
}

void parseArgument(char *arg)
{
    int option;
    char buf[1000];
    float foption;

    if (1 == sscanf(arg, "preset=%d", &option))
    {
        settingsDefault(option);
        return;
    }

    if (1 == sscanf(arg, "mode=%d", &option))
    {
        mode = option;
        if (option == 0)
        {
            printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
        }
        if (option == 1)
        {
            printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
        }
        if (option == 2)
        {
            printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd = 3;
        }
        return;
    }

    if (1 == sscanf(arg, "sampleoutput=%d", &option))
    {
        if (option == 1)
        {
            useSampleOutput = true;
            printf("USING SAMPLE OUTPUT WRAPPER!\n");
        }
        return;
    }

    if (1 == sscanf(arg, "quiet=%d", &option))
    {
        if (option == 1)
        {
            setting_debugout_runquiet = true;
            printf("QUIET MODE, I'll shut up!\n");
        }
        return;
    }

    if (1 == sscanf(arg, "nolog=%d", &option))
    {
        if (option == 1)
        {
            setting_logStuff = false;
            printf("DISABLE LOGGING!\n");
        }
        return;
    }

    if (1 == sscanf(arg, "nogui=%d", &option))
    {
        if (option == 1)
        {
            disableAllDisplay = true;
            printf("NO GUI!\n");
        }
        return;
    }
    if (1 == sscanf(arg, "nomt=%d", &option))
    {
        if (option == 1)
        {
            multiThreading = false;
            printf("NO MultiThreading!\n");
        }
        return;
    }
    if (1 == sscanf(arg, "calib=%s", buf))
    {
        calib = buf;
        printf("loading calibration from %s!\n", calib.c_str());
        return;
    }
    if (1 == sscanf(arg, "vignette=%s", buf))
    {
        vignetteFile = buf;
        printf("loading vignette from %s!\n", vignetteFile.c_str());
        return;
    }

    if (1 == sscanf(arg, "gamma=%s", buf))
    {
        gammaFile = buf;
        printf("loading gammaCalib from %s!\n", gammaFile.c_str());
        return;
    }

    if (1 == sscanf(arg, "linewidth=%f", &foption))
    {
        lw = foption;
        printf("ros traj linewidth %f!\n", lw);
        return;
    }
    printf("could not parse argument \"%s\"!!\n", arg);
}

// ==================== system definition =================
// FullSystem *fullSystem = 0;
std::shared_ptr<FullSystem> fullSystem(0);
Undistort *undistorter = 0;
int frameID = 0;
ImageAndExposure undistImg;
ImageAndExposure undistImg_right;
bool flag = false;

void callback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::ImageConstPtr &img_right)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    cv_bridge::CvImagePtr cv_ptr_right = cv_bridge::toCvCopy(img_right, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr_right->image.type() == CV_8U);
    assert(cv_ptr_right->image.channels() == 1);

    // MinimalImageB* minImg_left = new MinimalImageB(cv_)

    // MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows, (unsigned char *)cv_ptr->image.data);
    // MinimalImageB minImg_right((int)cv_ptr_right->image.cols, (int)cv_ptr_right->image.rows, (unsigned char *)cv_ptr_right->image.data);

    MinimalImageB *minImg = new MinimalImageB((int)cv_ptr->image.cols, (int)cv_ptr->image.rows);
    memcpy(minImg->data, cv_ptr->image.data, (int)cv_ptr->image.cols * (int)cv_ptr->image.rows);

    MinimalImageB *minImg_right = new MinimalImageB((int)cv_ptr_right->image.cols, (int)cv_ptr_right->image.rows);
    memcpy(minImg_right->data, cv_ptr_right->image.data, (int)cv_ptr_right->image.cols * (int)cv_ptr_right->image.rows);

    undistImg = *undistorter->undistort<unsigned char>(minImg, 1, 0, 1.0f);
    undistImg_right = *undistorter->undistort<unsigned char>(minImg_right, 1, 0, 1.0f);
    flag = true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "stereo_dso_ros");

    // ==================== parse args =====================
    for (int i = 1; i < argc; i++)
        parseArgument(argv[i]);

    // ===================== set calibration and distortion ================
    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
        (int)undistorter->getSize()[0],
        (int)undistorter->getSize()[1],
        undistorter->getK().cast<float>());

    baseline = undistorter->getBl();
    //setBaseline();

    // ======================init system========================
    fullSystem = std::shared_ptr<FullSystem>(new FullSystem());
    fullSystem->linearizeOperation = false;

    if (undistorter->photometricUndist != 0)
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    ros::NodeHandle nh;
    if (useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());
    else
        fullSystem->outputWrapper.push_back(
            new IOWrap::ROSOutputWrapper("/pointCloudMap", "/camPose", "/camTraj", "map", lw, nh));

    /*************************create and sync subscribers*****************************/
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/module/image_left", 1);   // "/camera/left/image_raw"
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/module/image_right", 1); // "/camera/right/image_raw"
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(1), left_sub, right_sub);
    sync.registerCallback(callback);
    /**********************************************************************/

    ros::Rate r(30);
    while (ros::ok())
    {
        // do slam if a pair of image received
        ROS_INFO_ONCE("Waiting for images!!");
        if (flag)
        {
            printf("new image received!\n");
            printf("frameID: %d\n", frameID);
            fullSystem->addActiveFrame(&undistImg, &undistImg_right, frameID);
            frameID++;
            // delete undistImg;
            // delete undistImg_right;
            // undistImg = nullptr;
            // undistImg_right = nullptr;
            flag = false;
        }
        if (setting_fullResetRequested || fullSystem->initFailed)
        {
            ROS_INFO("Resetting system!!");
            std::vector<IOWrap::Output3DWrapper *> wraps = fullSystem->outputWrapper;
            // delete fullSystem;
            fullSystem.reset();
            for (IOWrap::Output3DWrapper *ow : wraps)
                ow->reset();
            fullSystem = std::shared_ptr<FullSystem>(new FullSystem());
            fullSystem->linearizeOperation = false;
            fullSystem->outputWrapper = wraps;
            if (undistorter->photometricUndist != 0)
                fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
            setting_fullResetRequested = false;
        }

        if (fullSystem->isLost)
        {
            printf("LOST!!\n");
            break;
        }
        ros::spinOnce();
        r.sleep();
    }

    fullSystem->blockUntilMappingIsFinished();
    fullSystem->printResult("~/result.txt");

    for (IOWrap::Output3DWrapper *ow : fullSystem->outputWrapper)
    {
        ow->saveResults("~/results.bag");
        ow->join();
        delete ow;
    }
    delete undistorter;
    // delete fullSystem;
    printf("Delete fullsystem!\n");
    fullSystem.reset();
    printf("Exit now!\n");

    return 0;
}
