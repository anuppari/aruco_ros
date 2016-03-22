#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <aruco_ros/Center.h>
#include <aruco_ros/SpecialMarkers.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <typeinfo>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <math.h>
#include <aruco/aruco.h>
//#include <aruco/cvdrawingutils.h>

void rvec2quat(cv::Mat&, cv::Mat&);

double markerSize;
bool drawMarkers;

// need a class in order publish in the callback
class SubscribeAndPublish
{
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    ros::Subscriber camInfoSub;
    image_transport::Subscriber imageSub;
    ros::Publisher markerPosePub;
    ros::Publisher markerPointPub;
    ros::Publisher markerImagePub;
    tf::TransformBroadcaster tfBr;
    std::string cameraName;
    std::string image_frame_id;
    aruco::MarkerDetector MDetector;
    ros::ServiceServer service;
    
    // adaptive ROI
    bool adaptiveROI;
    int ROItop;
    int ROIleft;
    int ROIwidth;
    int ROIheight;
    double adaptiveROIfactor;
    
    bool gotCamParam;
    int imageWidth;
    int imageHeight;
    cv::Mat camMat;
    cv::Mat distCoeffs;
    
    std::map<int, bool> specialMarkersFound;
public:
    SubscribeAndPublish() : it(n)
    {
        //Camera name and marker size parameters
        ros::NodeHandle nh("~");
        nh.param<std::string>("camera", cameraName, "camera");
        nh.param<std::string>("image_frame_id", image_frame_id, "image");
        nh.param<double>("markerSize", markerSize, 0.2032);
        nh.param<bool>("drawMarkers", drawMarkers, true);
        nh.param<bool>("adaptiveROI", adaptiveROI, true);
        
        // Start service
        service = nh.advertiseService("setSpecialMarkers", &SubscribeAndPublish::setSpecialMarkers,this);
        
        //Adjust marker detector parameters
        if (adaptiveROI) {MDetector.setMinMaxSize(0.01,1.0);}
        else {MDetector.setMinMaxSize(0.01,0.9);}
        
        // marker pose and center publishers
        markerPosePub = n.advertise<geometry_msgs::PoseStamped>("markers",10);
        markerPointPub = n.advertise<aruco_ros::Center>("markerCenters",10);
        
        //Publisher for image with marker outlines
        if (drawMarkers){
            markerImagePub = n.advertise<sensor_msgs::Image>("markerImage",10);
        }
        
        // Get camera parameters
        std::cout << "Getting camera parameters on topic: "+cameraName+"/camera_info" << std::endl;
        gotCamParam = false;
        camInfoSub = nh.subscribe(cameraName+"/camera_info",1,&SubscribeAndPublish::camInfoCB,this);
        ROS_DEBUG("Waiting for camera parameters ...");
        do {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        } while (!(ros::isShuttingDown()) and !gotCamParam);
        ROS_DEBUG("Got camera parameters");
        
        // Set image ROI
        adaptiveROIfactor = 0.25;
        ROIleft = 0;
        ROItop = 0;
        ROIwidth = imageWidth;
        ROIheight = imageHeight;
        
        // Image and camera parameter subscribers
        imageSub = it.subscribe(cameraName+"/image_raw", 1, &SubscribeAndPublish::imageCb,this);
    }
    
    // callback for getting camera intrinsic parameters
    void camInfoCB(const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
    {
        //get camera info
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camInfoMsg);
        cv::Mat camMatCV = cv::Mat(cam_model.fullIntrinsicMatrix());
        camMatCV.convertTo(camMat,CV_64F);
        imageHeight = camInfoMsg->height;
        imageWidth = camInfoMsg->width;
        
        //unregister subscriber
        camInfoSub.shutdown();
        gotCamParam = true;
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& imageMsg)
    {
        //Convert to opencv image
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat image;
        
        try
        {
            cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
            image = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        try
        {
            cv::Mat imageROI = image(cv::Rect(ROIleft,ROItop,ROIwidth,ROIheight));
            
            //ros::Time start1 = ros::Time::now();
            // Adaptive ROI
            if (adaptiveROI)
            {
                // Adjust camera matrix
                camMat.at<double>(0,2) -= ROIleft;
                camMat.at<double>(1,2) -= ROItop;
                
                // draw ROI
                //cv::rectangle(image,cv::Point2d(ROIleft,ROItop),cv::Point2d(ROIleft+ROIwidth-1,ROItop+ROIheight-1),cv::Scalar(0,255,0));
            }
            //ros::Time end1 = ros::Time::now();
            //std::cout << "delt1: " << (end1-start1).toSec() << std::endl;
            
            //ros::Time start2 = ros::Time::now();
            //Detection of markers in the image passed
            vector<aruco::Marker> TheMarkers;
            MDetector.detect(imageROI,TheMarkers,camMat,distCoeffs,markerSize);
            //ros::Time end2 = ros::Time::now();
            //std::cout << "delt2: " << (end2-start2).toSec() << std::endl;
            
            // reset list of found special markers
            for (std::map<int,bool>::iterator it = specialMarkersFound.begin(); it != specialMarkersFound.end(); ++it)
            {
                it->second = false;
            }
            
            // generate pose message and tf broadcast
            if (TheMarkers.size()!=0){
                
                //Get image timestamp for subsequent publications
                ros::Time timeNow = imageMsg->header.stamp;
                
                // reset ROI
                int newROIleft = imageWidth;
                int newROItop = imageHeight;
                int newROIbottom = 0;
                int newROIright = 0;
                
                //ros::Time start3 = ros::Time::now();
                // Publish
                for (unsigned int i=0; i<TheMarkers.size(); i++) {
                    //Common Info
                    char buffer[4];
                    sprintf(buffer,"%d",TheMarkers[i].id);
                    
                    // Update special marker list
                    std::map<int,bool>::iterator it = specialMarkersFound.find(TheMarkers[i].id);
                    if (it != specialMarkersFound.end())
                    {
                        it->second = true;
                    }
                    
                    //Marker pose
                    cv::Mat tvec = TheMarkers[i].Tvec;
                    cv::Mat quat(4,1,CV_32FC1);
                    rvec2quat(TheMarkers[i].Rvec,quat);
                    
                    //Broadcast tf
                    tf::Quaternion q(quat.at<float>(1,0),quat.at<float>(2,0),quat.at<float>(3,0),quat.at<float>(0,0));
                    tf::Vector3 vec(tvec.at<float>(0,0),tvec.at<float>(1,0),tvec.at<float>(2,0));
                    tf::Transform transf(q,vec);
                    /*
                    char frameName[16];
                    sprintf(frameName,"marker%d",TheMarkers[i].id);
                    */
                    tfBr.sendTransform(tf::StampedTransform(transf,timeNow,image_frame_id,std::string("marker")+buffer));
                    
                    //Publish marker pose
                    geometry_msgs::PoseStamped poseMsg;
                    poseMsg.header.stamp = timeNow;
                    poseMsg.header.frame_id = buffer;
                    
                    poseMsg.pose.position.x = tvec.at<float>(0,0);
                    poseMsg.pose.position.y = tvec.at<float>(1,0);
                    poseMsg.pose.position.z = tvec.at<float>(2,0);
                    
                    poseMsg.pose.orientation.w = quat.at<float>(0,0);
                    poseMsg.pose.orientation.x = quat.at<float>(1,0);
                    poseMsg.pose.orientation.y = quat.at<float>(2,0);
                    poseMsg.pose.orientation.z = quat.at<float>(3,0);
                    markerPosePub.publish(poseMsg);
                    
                    //Publish marker centers
                    cv::Point2f cent = TheMarkers[i].getCenter();
                    aruco_ros::Center centerMsg;
                    centerMsg.header.stamp = timeNow;
                    centerMsg.header.frame_id = buffer;
                    centerMsg.x = cent.x + ROIleft;
                    centerMsg.y = cent.y + ROItop;
                    centerMsg.found = true;
                    markerPointPub.publish(centerMsg);
                    
                    // Draw marker on image
                    if (drawMarkers){
                        TheMarkers[i].draw(imageROI,cv::Scalar(0,0,255));
                    }
                    
                    // Adaptive ROI
                    for (unsigned int j=0; j<4; j++) {
                        newROIleft = std::min((int) TheMarkers[i][j].x + ROIleft,newROIleft);
                        newROItop = std::min((int) TheMarkers[i][j].y + ROItop,newROItop);
                        newROIright = std::max((int) TheMarkers[i][j].x + ROIleft,newROIright);
                        newROIbottom = std::max((int) TheMarkers[i][j].y + ROItop,newROIbottom);
                        //cv::circle(image,cv::Point2d(TheMarkers[i][j].x + ROIleft,TheMarkers[i][j].y + ROItop),10,cv::Scalar(255,0,0),-1);
                    }
                    //cv::rectangle(image,cv::Point2d(newROIleft,newROItop),cv::Point2d(newROIright-1,newROIbottom-1),cv::Scalar(255,0,0));
                }
                //ros::Time end3 = ros::Time::now();
                //std::cout << "delt3: " << (end3-start3).toSec() << std::endl;
                
                //ros::Time start4 = ros::Time::now();
                // Adjust ROI
                if (adaptiveROI)
                {
                    // Reset cam matrix
                    camMat.at<double>(0,2) += ROIleft;
                    camMat.at<double>(1,2) += ROItop;
                    
                    double maxWidthHeight = std::max(newROIright - newROIleft,newROIbottom - newROItop);
                    ROIleft = std::max(0,newROIleft - (int) (adaptiveROIfactor*maxWidthHeight));
                    ROItop = std::max(0,newROItop - (int) (adaptiveROIfactor*maxWidthHeight));
                    ROIwidth = std::min(imageWidth - ROIleft, (int) ((1+2*adaptiveROIfactor)*maxWidthHeight));
                    ROIheight = std::min(imageHeight - ROItop, (int) ((1+2*adaptiveROIfactor)*maxWidthHeight));
                }
                //ros::Time end4 = ros::Time::now();
                //std::cout << "delt4: " << (end4-start4).toSec() << std::endl;
            }
            else {
                // Reset ROI
                ROIleft = 0;
                ROItop = 0;
                ROIwidth = imageWidth;
                ROIheight = imageHeight;
            }
            //ros::Time start5 = ros::Time::now();
            // Publish image with marker outlines
            if (drawMarkers){
                markerImagePub.publish(cv_ptr->toImageMsg());
            }
            //ros::Time end5 = ros::Time::now();
            //std::cout << "delt5: " << (end5-start5).toSec() << std::endl;
            
            // Send out signal if special markers not found
            for (std::map<int,bool>::iterator it = specialMarkersFound.begin(); it != specialMarkersFound.end(); ++it)
            {
                if (it->second == false)
                {
                    //Publish marker centers
                    char buffer[4];
                    sprintf(buffer,"%d",it->first);
                    aruco_ros::Center centerMsg;
                    centerMsg.header.stamp = imageMsg->header.stamp;
                    centerMsg.header.frame_id = buffer;
                    centerMsg.found = false;
                    markerPointPub.publish(centerMsg);
                }
            }
        }
        catch (std::exception &ex){
            cout<<"Exception :"<<ex.what()<<endl;
        }
        //std::cout << std::endl << std::endl << std::endl;
    }
    
    bool setSpecialMarkers(aruco_ros::SpecialMarkers::Request &req,aruco_ros::SpecialMarkers::Response &resp)
    {
        // reset map of special markers
        if (req.reset) { specialMarkersFound.clear(); }
        
        // Initialize list of new special markers
        for (std::vector<int>::iterator it = req.markerIDs.begin(); it != req.markerIDs.end(); ++it)
        {
            specialMarkersFound[*it] = false;
        }
        
        return true;
    }


};//End of class SubscribeAndPublish


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_ros");
    
    SubscribeAndPublish sap;
    
    ros::spin();
    return 0;
}

void rvec2quat(cv::Mat &rvec, cv::Mat &quat)
{
    float theta = sqrt(pow(rvec.at<float>(0,0),2) + pow(rvec.at<float>(1,0),2) + pow(rvec.at<float>(2,0),2));
    cv::Mat unit = rvec/theta;
    quat.at<float>(0,0) = cos(theta/2);
    quat.at<float>(1,0) = sin(theta/2)*unit.at<float>(0,0);
    quat.at<float>(2,0) = sin(theta/2)*unit.at<float>(1,0);
    quat.at<float>(3,0) = sin(theta/2)*unit.at<float>(2,0);
}

