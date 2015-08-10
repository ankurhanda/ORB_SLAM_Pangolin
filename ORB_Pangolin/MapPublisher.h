/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include<ros/ros.h>
#include <pangolin/pangolin.h>

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"

namespace ORB_SLAM
{

class MapPublisher
{
public:
    MapPublisher(Map* pMap);

    Map* mpMap;

    void DrawMapPoints();
    void DrawKeyFrames();
    void DrawCurrentCamera();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

    cv::Mat GetCurrentCameraPose();

    cv::Mat mCameraPose;

    boost::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H
