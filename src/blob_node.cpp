#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <aruco_ros/Center.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <stdio.h>

bool drawMarkers;

// need a class in order publish in the callback
class SubscribeAndPublish
{
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber cam_sub;
    ros::Publisher markerPointPub;
    ros::Publisher markerImagePub;
    cv::SimpleBlobDetector detector;
    std::string cameraName;
public:
    SubscribeAndPublish() : it(n)
    {
        //Camera name and marker size parameters
        n.param<std::string>(ros::this_node::getName()+"/camera", cameraName, "camera");
        n.param<bool>(ros::this_node::getName()+"/drawMarkers", drawMarkers, true);
        
        // marker pose and center publishers
        markerPointPub = n.advertise<aruco_ros::Center>("markerCenters",10);
        
        //Publisher for image with marker outlines
        if (drawMarkers){
            markerImagePub = n.advertise<sensor_msgs::Image>("markerImage",10);
        }
        
        // Image and camera parameter subscribers
        cam_sub = it.subscribeCamera(cameraName+"/image_raw", 1, &SubscribeAndPublish::imageCb,this);
        
        //detector params
        cv::SimpleBlobDetector::Params params;
        params.minThreshold = 200;
        params.maxThreshold = 255;
        params.filterByArea = false;
        params.filterByCircularity = false;
        params.filterByConvexity = false;
        params.filterByInertia = false;
        params.filterByColor = true;
        params.blobColor = 255;
        detector = cv::SimpleBlobDetector(params);
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
        
        try
        {
            // Convert to Hue saturation value
            cv::Mat imageHSV;
            cv::cvtColor(image,imageHSV,CV_BGR2HSV);
            
            //Threshold
            cv::Mat imageThresh;
            cv::inRange(imageHSV,cv::Scalar(0,150,150),cv::Scalar(10,255,255),imageThresh);
            
            //Clean up image (remove noise and holes)
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5),cv::Point(3,3));
            cv::morphologyEx(imageThresh,imageThresh,cv::MORPH_OPEN,element);
            cv::morphologyEx(imageThresh,imageThresh,cv::MORPH_CLOSE,element);
            
            cv::Mat kptImage;
            std::vector<cv::KeyPoint> keypoints;
            detector.detect(imageThresh,keypoints);
            
            // publish center
            if (keypoints.size()>0){
                //Find keypoint with biggest area
                float maxArea = keypoints[0].size;
                int maxKyptInd = 0;
                for (int i=1; i<keypoints.size(); i++) {
                    if (keypoints[i].size > maxArea) {
                        maxArea = keypoints[i].size;
                        maxKyptInd = i;
                    }
                }
                
                //Get image timestamp for subsequent publications
                ros::Time timeNow = imageMsg->header.stamp;
                
                //Publish marker centers
                cv::Point2f cent = keypoints[maxKyptInd].pt;
                aruco_ros::Center centerMsg;
                centerMsg.header.stamp = timeNow;
                centerMsg.header.frame_id = "100";
                centerMsg.x = cent.x;
                centerMsg.y = cent.y;
                markerPointPub.publish(centerMsg);
                
                
                
                // Draw marker on image
                if (drawMarkers){
                    std::vector<cv::KeyPoint> biggestKeypoint (1);
                    biggestKeypoint[0] = keypoints[maxKyptInd];
                    cv::drawKeypoints(imageThresh,biggestKeypoint,kptImage,cv::Scalar(255,0,0));
                }
            }
            // Publish image with marker outlines
            if (drawMarkers){
                cv_bridge::CvImage imagePtr = cv_bridge::CvImage(imageMsg->header,"bgr8",kptImage);
                markerImagePub.publish(imagePtr.toImageMsg());
            }
        }
        catch (std::exception &ex){
            std::cout<<"Exception :"<<ex.what()<<std::endl;
        }
    }


};//End of class SubscribeAndPublish


int main(int argc, char** argv)
{
    ros::init(argc, argv, "blob_node");
    
    SubscribeAndPublish sap;
    
    ros::spin();
    return 0;
}

