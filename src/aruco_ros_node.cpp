#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <aruco_ros/Center.h>
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
    image_transport::CameraSubscriber cam_sub;
    ros::Publisher markerPosePub;
    ros::Publisher markerPointPub;
    ros::Publisher markerImagePub;
    tf::TransformBroadcaster tfBr;
    std::string cameraName;
    std::string image_frame_id;
    aruco::MarkerDetector MDetector;
public:
    SubscribeAndPublish() : it(n)
    {
        //Camera name and marker size parameters
        n.param<std::string>(ros::this_node::getName()+"/camera", cameraName, "camera");
        n.param<std::string>(ros::this_node::getName()+"/image_frame_id", image_frame_id, "image");
        n.param<double>(ros::this_node::getName()+"/markerSize", markerSize, 0.2032);
        n.param<bool>(ros::this_node::getName()+"/drawMarkers", drawMarkers, true);
        
        //Adjust marker detector parameters
        MDetector.setMinMaxSize(0.01,0.9);
        
        // marker pose and center publishers
        markerPosePub = n.advertise<geometry_msgs::PoseStamped>("markers",10);
        markerPointPub = n.advertise<aruco_ros::Center>("markerCenters",10);
        
        //Publisher for image with marker outlines
        if (drawMarkers){
            markerImagePub = n.advertise<sensor_msgs::Image>("markerImage",10);
        }
        
        // Image and camera parameter subscribers
        cam_sub = it.subscribeCamera(cameraName+"/image_raw", 1, &SubscribeAndPublish::imageCb,this);
        
        
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
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
        
        //get camera info
        image_geometry::PinholeCameraModel cam_model;
        cam_model.fromCameraInfo(camInfoMsg);
        cv::Mat camMat = cv::Mat(cam_model.fullIntrinsicMatrix());
        camMat.convertTo(camMat,CV_32FC1);
        cv::Mat distCoeffs;
        cam_model.distortionCoeffs().convertTo(distCoeffs,CV_32FC1);
        if (cv::countNonZero(camMat) < 1) // check if camera parameters are default values used when not set by camera driver, and serve empty Mat to detector
        {
            camMat = cv::Mat();
            distCoeffs = cv::Mat();
        }
        
        try
        {
            //Detection of markers in the image passed
            vector<aruco::Marker> TheMarkers;
            MDetector.detect(image,TheMarkers,camMat,distCoeffs,markerSize);
            
            // generate pose message and tf broadcast
            if (TheMarkers.size()!=0){
                
                //Get image timestamp for subsequent publications
                ros::Time timeNow = imageMsg->header.stamp;
                
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
                    centerMsg.x = cent.x;
                    centerMsg.y = cent.y;
                    markerPointPub.publish(centerMsg);
                    
                    // Draw marker on image
                    if (drawMarkers){
                        TheMarkers[i].draw(image,cv::Scalar(0,0,255));
                    }
                }
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

