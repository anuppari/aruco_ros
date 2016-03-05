#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <aruco_ros/Center.h>
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
        
        //Adjust marker detector parameters
        MDetector.setMinMaxSize(0.01,0.9);
        
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
        adaptiveROIfactor = 0.1;
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
            
            // Adaptive ROI
            if (adaptiveROI)
            {
                // Adjust camera matrix
                camMat.at<double>(0,2) -= ROIleft;
                camMat.at<double>(1,2) -= ROItop;
                
                // draw ROI
                cv::rectangle(image,cv::Point2d(ROIleft,ROItop),cv::Point2d(ROIleft+ROIwidth-1,ROItop+ROIheight-1),cv::Scalar(0,255,0));
            }
            
            //Detection of markers in the image passed
            vector<aruco::Marker> TheMarkers;
            MDetector.detect(imageROI,TheMarkers,camMat,distCoeffs,markerSize);
            
            // generate pose message and tf broadcast
            if (TheMarkers.size()!=0){
                
                //Get image timestamp for subsequent publications
                ros::Time timeNow = imageMsg->header.stamp;
                
                // reset ROI
                int newROIleft = imageWidth;
                int newROItop = imageHeight;
                int newROIbottom = 0;
                int newROIright = 0;
                
                // Publish
                for (unsigned int i=0; i<TheMarkers.size(); i++) {
                    //Common Info
                    char buffer[4];
                    sprintf(buffer,"%d",TheMarkers[i].id);
                    
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
                    markerPointPub.publish(centerMsg);
                    
                    // Draw marker on image
                    if (drawMarkers){
                        TheMarkers[i].draw(imageROI,cv::Scalar(0,0,255));
                    }
                    
                    // Adaptive ROI
                    for (unsigned int j=0; j<4; j++) {
                        newROIleft = std::min((int) TheMarkers[i][j].x,newROIleft);
                        newROItop = std::min((int) TheMarkers[i][j].y,newROItop);
                        newROIright = std::max((int) TheMarkers[i][j].x,newROIright);
                        newROIbottom = std::max((int) TheMarkers[i][j].y,newROIbottom);
                    }
                }
                
                // Adjust ROI
                if (adaptiveROI)
                {
                    // Reset cam matrix
                    camMat.at<double>(0,2) += ROIleft;
                    camMat.at<double>(1,2) += ROItop;
                    
                    ROIleft = std::max(0,newROIleft - (int) (adaptiveROIfactor*(newROIright - newROIleft)));
                    ROItop = std::max(0,newROItop - (int) (adaptiveROIfactor*(newROIbottom - newROItop)));
                    ROIwidth = std::min(imageWidth - ROIleft, (int) (2*adaptiveROIfactor*(newROIright - newROIleft)));
                    ROIheight = std::min(imageHeight - ROItop, (int) (2*adaptiveROIfactor*(newROIright - newROIleft)));
                }
            }
            else {
                // Reset ROI
                ROIleft = 0;
                ROItop = 0;
                ROIwidth = imageWidth;
                ROIheight = imageHeight;
            }
            // Publish image with marker outlines
            if (drawMarkers){
                markerImagePub.publish(cv_ptr->toImageMsg());
            }
        }
        catch (std::exception &ex){
            cout<<"Exception :"<<ex.what()<<endl;
        }
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

