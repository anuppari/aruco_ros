#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <aruco_ros/Center.h>
#include <aruco_ros/SpecialMarkers.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <math.h>

void rvec2quat(cv::Vec3d&, cv::Mat&);

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
    ros::ServiceServer service;
    
    // Parameters
    double markerSize;
    bool drawMarkers;
    //cv::aruco::Dictionary markerDictionary;
    //cv::aruco::DetectorParameters detectorParams;
    
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
        camMat = cv::Mat();
        distCoeffs = cv::Mat();
        
        // Get camera parameters
        std::cout << "Getting camera parameters on topic: "+cameraName+"/camera_info" << std::endl;
        gotCamParam = false;
        camInfoSub = n.subscribe(cameraName+"/camera_info",1,&SubscribeAndPublish::camInfoCB,this);
        
        // Start service
        service = nh.advertiseService("setSpecialMarkers", &SubscribeAndPublish::setSpecialMarkers,this);
        
        //Adjust marker detector parameters
        //markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        //detectorParams = cv::aruco::DetectorParameters();
        //detectorParams.doCornerRefinement = true;
        
        // marker pose and center publishers
        markerPosePub = n.advertise<geometry_msgs::PoseStamped>("markers",10);
        markerPointPub = n.advertise<aruco_ros::Center>("markerCenters",10);
        
        //Publisher for image with marker outlines
        if (drawMarkers){
            markerImagePub = n.advertise<sensor_msgs::Image>("markerImage",10);
        }
        
        // Set ROI parameters
        adaptiveROIfactor = 0.25;
        
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
        cam_model.distortionCoeffs().convertTo(distCoeffs,CV_64F);
        if (cv::countNonZero(camMat) < 1) // check if camera parameters are default values used when not set by camera driver, and serve empty Mat to detector
        {
            camMat = cv::Mat();
            distCoeffs = cv::Mat();
        }
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
            // Reset ROI on first call
            if (!(0 <= ROIleft && 0 <= ROIwidth && ROIleft + ROIwidth <= image.cols && 0 <= ROItop && 0 <= ROIheight && ROItop + ROIheight <= image.rows))
            {
                imageWidth = image.cols;
                imageHeight = image.rows;
                
                ROIleft = 0;
                ROItop = 0;
                ROIwidth = imageWidth;
                ROIheight = imageHeight;
            }
            cv::Mat imageROI = image(cv::Rect(ROIleft,ROItop,ROIwidth,ROIheight));
            cv::Mat camMatROI = camMat.clone();
            
            // Adaptive ROI
            if (adaptiveROI)
            {
                if (gotCamParam)
                {
                    // Adjust camera matrix
                    camMatROI.at<double>(0,2) -= ROIleft;
                    camMatROI.at<double>(1,2) -= ROItop;
                }
                
                // draw ROI
                cv::rectangle(image,cv::Point2d(ROIleft,ROItop),cv::Point2d(ROIleft+ROIwidth-1,ROItop+ROIheight-1),cv::Scalar(0,255,0));
            }
            
            cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
            cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
            detectorParams->doCornerRefinement = true;
            
            //Detection of markers in the image passed
            std::cout << "here1" << std::endl;
            std::vector< std::vector<cv::Point2d> > markerCorners;
            std::vector<int> markerIDs;
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::Mat testImage = cv::imread("/home/ncr/frame.jpg");
            cv::imshow("window",testImage);
            cv::waitKey(10);
            cv::aruco::detectMarkers(testImage,markerDictionary,markerCorners,markerIDs,detectorParams);
            std::cout << "here2" << std::endl;
            
            // Determine pose, draw markers, publish pose and tf messages
            if (markerIDs.size() > 0)
            {
                //Get image timestamp for subsequent publications
                ros::Time timeNow = imageMsg->header.stamp;
                
                // Determine pose
                if (gotCamParam)
                {
                    cv::aruco::estimatePoseSingleMarkers(markerCorners,markerSize,camMat,distCoeffs,rvecs,tvecs);
                }
                std::cout << "here3" << std::endl;
                
                // reset ROI
                int newROIleft = imageWidth;
                int newROItop = imageHeight;
                int newROIbottom = 0;
                int newROIright = 0;
                    
                for (int i = 0; i < markerIDs.size(); i++)
                {
                    //Common Info
                    char buffer[4];
                    sprintf(buffer,"%d",markerIDs[i]);
                    
                    // Update special marker list
                    std::map<int,bool>::iterator it = specialMarkersFound.find(markerIDs[i]);
                    if (it != specialMarkersFound.end())
                    {
                        it->second = true;
                    }
                    
                    //Publish marker centers
                    cv::Point2d cent(0,0);
                    for (int j = 0; j < 4; j++)
                    {
                        cent += markerCorners[i][j];
                    }
                    cent *= 0.25;
                    aruco_ros::Center centerMsg;
                    centerMsg.header.stamp = timeNow;
                    centerMsg.header.frame_id = buffer;
                    centerMsg.x = cent.x + ROIleft;
                    centerMsg.y = cent.y + ROItop;
                    centerMsg.found = true;
                    markerPointPub.publish(centerMsg);
                    
                    // Publish pose
                    if (gotCamParam)
                    {
                        //Marker pose
                        cv::Mat quat(4,1,CV_64F);
                        rvec2quat(rvecs[i],quat);
                        
                        //Broadcast tf
                        tf::Quaternion q(quat.at<float>(1,0),quat.at<float>(2,0),quat.at<float>(3,0),quat.at<float>(0,0));
                        tf::Vector3 vec(tvecs[i][0,0],tvecs[i][1,0],tvecs[i][2,0]);
                        tf::Transform transf(q,vec);
                        tfBr.sendTransform(tf::StampedTransform(transf,timeNow,image_frame_id,std::string("marker")+buffer));
                        
                        //Publish marker pose
                        geometry_msgs::PoseStamped poseMsg;
                        poseMsg.header.stamp = timeNow;
                        poseMsg.header.frame_id = buffer;
                        
                        poseMsg.pose.position.x = tvecs[i][0,0];
                        poseMsg.pose.position.y = tvecs[i][1,0];
                        poseMsg.pose.position.z = tvecs[i][2,0];
                        
                        poseMsg.pose.orientation.w = quat.at<float>(0,0);
                        poseMsg.pose.orientation.x = quat.at<float>(1,0);
                        poseMsg.pose.orientation.y = quat.at<float>(2,0);
                        poseMsg.pose.orientation.z = quat.at<float>(3,0);
                        markerPosePub.publish(poseMsg);
                    }
                    
                    // Adaptive ROI
                    for (unsigned int j=0; j<4; j++) {
                        newROIleft = std::min((int) markerCorners[i][j].x + ROIleft,newROIleft);
                        newROItop = std::min((int) markerCorners[i][j].y + ROItop,newROItop);
                        newROIright = std::max((int) markerCorners[i][j].x + ROIleft,newROIright);
                        newROIbottom = std::max((int) markerCorners[i][j].y + ROItop,newROIbottom);
                        //cv::circle(image,cv::Point2d(TheMarkers[i][j].x + ROIleft,TheMarkers[i][j].y + ROItop),10,cv::Scalar(255,0,0),-1);
                    }
                }
                std::cout << "here4" << std::endl;
                
                // Draw marker on image and publish
                if (drawMarkers)
                {
                    cv::aruco::drawDetectedMarkers(imageROI, markerCorners, markerIDs, cv::Scalar(0,0,255));
                    markerImagePub.publish(cv_ptr->toImageMsg());
                }
                
                // Update ROI
                if (adaptiveROI)
                {
                    double maxWidthHeight = std::max(newROIright - newROIleft,newROIbottom - newROItop);
                    newROIleft = (int) (newROIleft - (maxWidthHeight - (newROIright-newROIleft))/2.0);
                    newROItop = (int) (newROItop - (maxWidthHeight - (newROIbottom-newROItop))/2.0);
                    ROIleft = std::max(0,newROIleft - (int) (adaptiveROIfactor*maxWidthHeight));
                    ROItop = std::max(0,newROItop - (int) (adaptiveROIfactor*maxWidthHeight));
                    ROIwidth = std::min(imageWidth - ROIleft, (int) ((1+2*adaptiveROIfactor)*maxWidthHeight));
                    ROIheight = std::min(imageHeight - ROItop, (int) ((1+2*adaptiveROIfactor)*maxWidthHeight));
                }
                std::cout << "here5" << std::endl;
            }
            else {
                // Reset ROI
                ROIleft = 0;
                ROItop = 0;
                ROIwidth = imageWidth;
                ROIheight = imageHeight;
            }
            
            // reset list of found special markers
            for (std::map<int,bool>::iterator it = specialMarkersFound.begin(); it != specialMarkersFound.end(); ++it)
            {
                it->second = false;
            }
            
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
            std::cout << "here6" << std::endl;
        }
        catch (std::exception &ex){
            std::cout << "Exception :" << ex.what() << std::endl;
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

void rvec2quat(cv::Vec3d &rvec, cv::Mat &quat)
{
    float theta = sqrt(pow(rvec[0,0],2) + pow(rvec[1,0],2) + pow(rvec[2,0],2));
    cv::Mat unit = cv::Mat(rvec)/theta;
    quat.at<float>(0,0) = cos(theta/2);
    quat.at<float>(1,0) = sin(theta/2)*unit.at<float>(0,0);
    quat.at<float>(2,0) = sin(theta/2)*unit.at<float>(1,0);
    quat.at<float>(3,0) = sin(theta/2)*unit.at<float>(2,0);
}

