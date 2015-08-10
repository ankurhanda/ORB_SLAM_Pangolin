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

#include "MapPublisher.h"
#include "MapPoint.h"
#include "KeyFrame.h"

#include<pangolin/pangolin.h>

namespace ORB_SLAM
{


MapPublisher::MapPublisher(Map* pMap):mpMap(pMap)
{    
}


void MapPublisher::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(2);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,0.0);
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(2);
        glBegin(GL_POINTS);
        glColor3f(1.0,0.0,0.0);
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
    glEnd();

}

void MapPublisher::DrawKeyFrames()
{
    float w = 0.05;
    float h = w*0.9;
    float z = w*0.6;

    //Camera is a pyramid. Define in camera coordinate system

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        cv::Mat Twc = pKF->GetPoseInverse().t();

        glPushMatrix();

        glMultMatrixf(Twc.ptr<GLfloat>(0));

        glLineWidth(1);
        glColor3f(0.0f,0.0f,1.0f);

        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();

    }

    glLineWidth(0.5f);
    glColor4f(0.0f,1.0f,0.0f,0.4f);
    glBegin(GL_LINES);

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        // Covisibility Graph
        vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        cv::Mat Ow = vpKFs[i]->GetCameraCenter();
        if(!vCovKFs.empty())
        {
            for(vector<KeyFrame*>::iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
            {
                if((*vit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Ow2 = (*vit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
            }
        }

        // MST
        KeyFrame* pParent = vpKFs[i]->GetParent();
        if(pParent)
        {
            cv::Mat Owp = pParent->GetCameraCenter();
            glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
            glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
        }

        set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
        for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
        {
            if((*sit)->mnId<vpKFs[i]->mnId)
                continue;
            cv::Mat Owl = (*sit)->GetCameraCenter();
            glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
            glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
        }
    }

    glEnd();



}

void MapPublisher::DrawCurrentCamera()
{
    float w = 0.05;
    float h = w*0.9;
    float z = w*0.6;

    cv::Mat Tcw = GetCurrentCameraPose();
    if(Tcw.empty())
        return;

    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
    Twc.rowRange(0,3).colRange(0,3) = Rcw.t();
    Twc.rowRange(0,3).col(3) = -Rcw.t()*tcw;
    Twc = Twc.t();

    glPushMatrix();

    glMultMatrixf(Twc.ptr<GLfloat>(0));
    glLineWidth(3);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();

}

void MapPublisher::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    float w = 0.08;
    float h = w*0.75;
    float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(3);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();

}

void MapPublisher::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            boost::mutex::scoped_lock lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}


void MapPublisher::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

cv::Mat MapPublisher::GetCurrentCameraPose()
{
    boost::mutex::scoped_lock lock(mMutexCamera);
    return mCameraPose.clone();
}

} //namespace ORB_SLAM
