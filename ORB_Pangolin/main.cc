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

#include<iostream>
#include<fstream>
#include<ros/ros.h>
#include<ros/package.h>
#include<boost/thread.hpp>

#include<opencv2/core/core.hpp>

#include<pangolin/pangolin.h>

#include "Tracking.h"
#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"


#include "Converter.h"
#include "Optimizer.h"


using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ORB_SLAM");
    ros::start();

    cout << endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl;

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM ORB_SLAM path_to_vocabulary path_to_settings (absolute or relative to package directory)" << endl;
        ros::shutdown();
        return 1;
    }

    // Load Settings and Check
    string strSettingsFile = ros::package::getPath("ORB_SLAM")+"/"+argv[2];

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        ROS_ERROR("Wrong path to settings. Path must be absolut or relative to ORB_SLAM package directory.");
        ros::shutdown();
        return 1;
    }

    //Create Frame Publisher for image_view
    ORB_SLAM::FramePublisher FramePub;

    //Load ORB Vocabulary
    string strVocFile = ros::package::getPath("ORB_SLAM")+"/"+argv[1];
    cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    cv::FileStorage fsVoc(strVocFile.c_str(), cv::FileStorage::READ);
    if(!fsVoc.isOpened())
    {
        cerr << endl << "Wrong path to vocabulary. Path must be absolut or relative to ORB_SLAM package directory." << endl;
        ros::shutdown();
        return 1;
    }
    ORB_SLAM::ORBVocabulary Vocabulary;
    Vocabulary.load(fsVoc);

    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    //Create the map
    ORB_SLAM::Map World;

    FramePub.SetMap(&World);

    //Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&World);

    //Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile);
    boost::thread trackingThread(&ORB_SLAM::Tracking::Run,&Tracker);

    Tracker.SetKeyFrameDatabase(&Database);

    //Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&LocalMapper);

    //Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

    //Set pointers between threads
    Tracker.SetLocalMapper(&LocalMapper);
    Tracker.SetLoopClosing(&LoopCloser);

    LocalMapper.SetTracker(&Tracker);
    LocalMapper.SetLoopCloser(&LoopCloser);

    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);

    //OPENGL Visualization
    int image_width = fsSettings["Camera.width"];
    if(image_width==0)
        image_width = 640;
    int image_height = fsSettings["Camera.height"];
    if(image_height==0)
        image_height = 480;
    pangolin::CreateWindowAndBind("Main",1500,600);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::View& d_image = pangolin::Display("Frame")
            .SetBounds(0.0f,1.0f,pangolin::Attach::Pix(120),pangolin::Attach::Pix(120+image_width),(float)(image_width)/(image_height+20))
            .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::CreatePanel("ui")
            .SetBounds(0.5, 1.0, 0.0, pangolin::Attach::Pix(120));

    pangolin::Var<bool> ui_tracking("ui.Only Tracking",false,true);
    pangolin::Var<bool> ui_reset("ui.Reset",false,false);
    pangolin::Var<bool> ui_BA("ui.Full BA",false,false);
    pangolin::Var<bool> ui_follow("ui.Follow Camera",true,true);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
                pangolin::ModelViewLookAt(-1,-0.3,-3, -1,0,0, 0.0,-1.0,0.0)
                );

    pangolin::View& d_cam = pangolin::Display("World")
            .SetBounds(0.0f, 1.0f, pangolin::Attach::Pix(120), 1.0, -640.0f/480.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::GlTexture imageTexture(image_width,image_height+20);

    //This "main" thread will show the current processed frame and publish the map
    float fps = fsSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    ros::Rate r(fps);    

    bool bOnlyTracking = false;
    bool bFollow = true;
    cv::Mat im;
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    while (ros::ok() && !pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        if(ui_tracking && !bOnlyTracking)
        {
            if(World.KeyFramesInMap()>2)
            {
                LocalMapper.RequestStop();
                bOnlyTracking = true;
            }
            else
            {
                cout << "You cannot disable mapping, there is no map yet!" << endl;
                ui_tracking = false;
            }
        }
        else if(!ui_tracking && bOnlyTracking)
        {
            LocalMapper.Release();
            bOnlyTracking = false;
        }

        if(ui_BA)
        {
            if(!bOnlyTracking)
            {
                LocalMapper.RequestStop();
                ros::Rate rBA(200);
                while(!LocalMapper.isStopped())
                {
                    rBA.sleep();
                }
            }
            ORB_SLAM::Optimizer::GlobalBundleAdjustemnt(&World,5);

            if(!bOnlyTracking)
            {
                LocalMapper.Release();
            }

            ui_BA=false;
        }


        if(ui_reset)
        {
            Tracker.ResetFromGUI();
            ui_reset = false;
            ui_tracking=false;
            bOnlyTracking = false;
        }

        FramePub.DrawFrame(im);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        MapPub.GetCurrentOpenGLCameraMatrix(Twc);

        if(ui_follow && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(ui_follow && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(-1,-0.3,-3, -1,0,0, 0.0,-1.0,0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!ui_follow && bFollow)
        {
            bFollow = false;
        }


        d_cam.Activate(s_cam);

        MapPub.DrawMapPoints();
        MapPub.DrawKeyFrames();
        MapPub.DrawCurrentCamera(Twc);

        imageTexture.Reinitialise(im.cols,im.rows);
        imageTexture.Upload(im.ptr<unsigned char*>(0),GL_BGR,GL_UNSIGNED_BYTE);

        //display the image
        d_image.Activate();
        glColor3f(1.0,1.0,1.0);
        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();
        Tracker.CheckResetByPublishers();
        r.sleep();
    }

    // Save keyframe poses at the end of the execution
    ofstream f;

    vector<ORB_SLAM::KeyFrame*> vpKFs = World.GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM::KeyFrame::lId);

    cout << endl << "Saving Keyframe Trajectory to KeyFrameTrajectory.txt" << endl;
    string strFile = ros::package::getPath("ORB_SLAM")+"/"+"KeyFrameTrajectory.txt";
    f.open(strFile.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        ORB_SLAM::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = ORB_SLAM::Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }
    f.close();

    ros::shutdown();

	return 0;
}
