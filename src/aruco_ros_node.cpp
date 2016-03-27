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

void rvec2quat(cv::Vec3d&, tf::Quaternion&);

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
    cv::Ptr<cv::aruco::Dictionary> markerDictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    
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
    
    // debug
    cv::Mat lastImage;
    ros::Time lastImageTime;
    int doubleTimeCount;
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
        lastImage = cv::Mat();
        doubleTimeCount = 0;
        
        //Adjust marker detector parameters
        markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        detectorParams = cv::aruco::DetectorParameters::create();
        nh.getParam("adaptiveThreshWinSizeMin", detectorParams->adaptiveThreshWinSizeMin);
        nh.getParam("adaptiveThreshWinSizeMax", detectorParams->adaptiveThreshWinSizeMax);
        nh.getParam("adaptiveThreshWinSizeStep", detectorParams->adaptiveThreshWinSizeStep);
        nh.getParam("adaptiveThreshConstant", detectorParams->adaptiveThreshConstant);
        nh.getParam("minMarkerPerimeterRate", detectorParams->minMarkerPerimeterRate);
        nh.getParam("maxMarkerPerimeterRate", detectorParams->maxMarkerPerimeterRate);
        nh.getParam("polygonalApproxAccuracyRate", detectorParams->polygonalApproxAccuracyRate);
        nh.getParam("minCornerDistanceRate", detectorParams->minCornerDistanceRate);
        nh.getParam("minDistanceToBorder", detectorParams->minDistanceToBorder);
        nh.getParam("minMarkerDistanceRate", detectorParams->minMarkerDistanceRate);
        nh.param<bool>("doCornerRefinement", detectorParams->doCornerRefinement, true);
        nh.getParam("cornerRefinementWinSize", detectorParams->cornerRefinementWinSize);
        nh.getParam("cornerRefinementMaxIterations", detectorParams->cornerRefinementMaxIterations);
        nh.getParam("cornerRefinementMinAccuracy", detectorParams->cornerRefinementMinAccuracy);
        nh.getParam("markerBorderBits", detectorParams->markerBorderBits);
        nh.getParam("perspectiveRemovePixelPerCell", detectorParams->perspectiveRemovePixelPerCell);
        nh.getParam("perspectiveRemoveIgnoredMarginPerCell", detectorParams->perspectiveRemoveIgnoredMarginPerCell);
        nh.getParam("maxErroneousBitsInBorderRate", detectorParams->maxErroneousBitsInBorderRate);
        nh.getParam("minOtsuStdDev", detectorParams->minOtsuStdDev);
        nh.getParam("errorCorrectionRate", detectorParams->errorCorrectionRate);
        
        // Set ROI parameters
        adaptiveROIfactor = 0.25;
        ROIleft = -1;
        ROItop = -1;
        ROIwidth = -1;
        ROIheight = -1;
        
        // Get camera parameters
        camMat = cv::Mat();
        distCoeffs = cv::Mat();
        imageWidth = -1;
        imageHeight = -1;
        std::cout << "Getting camera parameters on topic: "+cameraName+"/camera_info" << std::endl;
        gotCamParam = false;
        camInfoSub = n.subscribe(cameraName+"/camera_info",1,&SubscribeAndPublish::camInfoCB,this);
        
        // Start service
        service = nh.advertiseService("setSpecialMarkers", &SubscribeAndPublish::setSpecialMarkers,this);
        
        // marker pose and center publishers
        markerPosePub = n.advertise<geometry_msgs::PoseStamped>("markers",10);
        markerPointPub = n.advertise<aruco_ros::Center>("markerCenters",10);
        
        //Publisher for image with marker outlines
        if (drawMarkers){
            markerImagePub = n.advertise<sensor_msgs::Image>("markerImage",10);
        }
        
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
        // Convert to opencv image
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
        
        // DEBUG
        //ros::Time timeNow = imageMsg->header.stamp;
        //double delT = timeNow.toSec() - lastImageTime.toSec();
        //if (delT == 0) //(delT == 0)
        //{
            //char buffer[2];
            //sprintf(buffer,"%d",doubleTimeCount);
            ////cv::imwrite(std::string("/home/ncr/testdiff/lastImage")+buffer+std::string(".jpg"),lastImage);
            ////cv::imwrite(std::string("/home/ncr/testdiff/thisImage")+buffer+std::string(".jpg"),image);
            ////cv::Mat diffImage;
            ////cv::absdiff(image,lastImage,diffImage);
            ////double min, max;
            ////cv::minMaxIdx(diffImage,&min,&max);
            ////cv::Mat diffImageScaled;
            ////cv::convertScaleAbs(diffImage,diffImageScaled,255.0/(max-min),-1.0*min);
            ////cv::imwrite(std::string("/home/ncr/testdiff/diffImage")+buffer+std::string(".jpg"),diffImage);
            ////cv::imwrite(std::string("/home/ncr/testdiff/diffImageScaled")+buffer+std::string(".jpg"),diffImageScaled);
            //std::cout.precision(18);
            //std::cout << "lastImageTime sec: " << lastImageTime.sec << std::endl;
            //std::cout << "lastImageTime nsec: " << lastImageTime.nsec << std::endl;
            //std::cout << "timeNow sec: " << timeNow.sec << std::endl;
            //std::cout << "timeNow nsec: " << timeNow.nsec << std::endl;
            //std::cout << "lastImageTime: " << lastImageTime.toSec() << std::endl;
            //std::cout << "timeNow: " << timeNow.toSec() << std::endl;
            //std::cout << "doubleTimeCount: " << doubleTimeCount << std::endl;
            //doubleTimeCount++;
        //}
        //lastImageTime = timeNow;
        //lastImage = image.clone();
        
        
        // Get image dimensions on first call if camera_info not called
        if (!(imageHeight > 0 && imageWidth > 0))
        {
            imageWidth = image.cols;
            imageHeight = image.rows;
        }
        
        // Reset ROI on first call
        if (!(ROIleft >= 0 && ROItop >= 0 && ROIwidth >=0 && ROIheight >= 0 && ROIleft + ROIwidth <= imageWidth && ROItop + ROIheight <= imageHeight))
        {
            ROIleft = 0;
            ROItop = 0;
            ROIwidth = imageWidth;
            ROIheight = imageHeight;
        }
        
        // Get ROI
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
        
        // Detect markers
        std::vector< std::vector<cv::Point2f> > markerCorners;
        std::vector<int> markerIDs;
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::detectMarkers(imageROI,markerDictionary,markerCorners,markerIDs,detectorParams);
        
        // Determine pose, draw markers, publish pose and tf messages
        if (markerIDs.size() > 0)
        {
            //Get image timestamp for subsequent publications
            ros::Time timeNow = imageMsg->header.stamp;
            
            // Determine pose
            if (gotCamParam) { cv::aruco::estimatePoseSingleMarkers(markerCorners,markerSize,camMatROI,distCoeffs,rvecs,tvecs); }
            
            // Reset temp variables for new ROI
            int newROIleft = imageWidth;
            int newROItop = imageHeight;
            int newROIbottom = 0;
            int newROIright = 0;
            
            for (int i = 0; i < markerIDs.size(); i++)
            {
                // Common Info
                char buffer[4];
                std::sprintf(buffer,"%d",markerIDs[i]);
                
                // Update special marker list
                std::map<int,bool>::iterator it = specialMarkersFound.find(markerIDs[i]);
                if (it != specialMarkersFound.end())
                {
                    it->second = true;
                }
                
                // Publish marker centers
                cv::Point2f cent(0,0);
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
                cv::circle(imageROI,cent,10,cv::Scalar(255,0,0),-1);
                
                // Publish pose
                if (gotCamParam)
                {
                    //Marker pose
                    tf::Quaternion quat;
                    rvec2quat(rvecs[i],quat);
                    //tf::Quaternion q(quat.at<double>(1,0),quat.at<double>(2,0),quat.at<double>(3,0),quat.at<double>(0,0));
                    tf::Quaternion qComp(0,0,std::sqrt(2.0)/2.0,-std::sqrt(2.0)/2.0);
                    quat *= qComp; // compensate for change in convention between Aruco and OpenCV/Aruco
                    
                    //Broadcast tf
                    tf::Vector3 vec(tvecs[i][0],tvecs[i][1],tvecs[i][2]);
                    tf::Transform transf(quat,vec);
                    tfBr.sendTransform(tf::StampedTransform(transf,timeNow,image_frame_id,std::string("marker")+buffer));
                    
                    //Publish marker pose
                    geometry_msgs::PoseStamped poseMsg;
                    poseMsg.header.stamp = timeNow;
                    poseMsg.header.frame_id = buffer;
                    
                    poseMsg.pose.position.x = tvecs[i][0];
                    poseMsg.pose.position.y = tvecs[i][1];
                    poseMsg.pose.position.z = tvecs[i][2];
                    
                    poseMsg.pose.orientation.x = quat.getX();
                    poseMsg.pose.orientation.y = quat.getY();
                    poseMsg.pose.orientation.z = quat.getZ();
                    poseMsg.pose.orientation.w = quat.getW();
                    markerPosePub.publish(poseMsg);
                }
                
                // Determine new ROI
                for (unsigned int j=0; j<4; j++) {
                    newROIleft = std::min((int) markerCorners[i][j].x + ROIleft,newROIleft);
                    newROItop = std::min((int) markerCorners[i][j].y + ROItop,newROItop);
                    newROIright = std::max((int) markerCorners[i][j].x + ROIleft,newROIright);
                    newROIbottom = std::max((int) markerCorners[i][j].y + ROItop,newROIbottom);
                    //cv::circle(image,cv::Point2d(TheMarkers[i][j].x + ROIleft,TheMarkers[i][j].y + ROItop),10,cv::Scalar(255,0,0),-1);
                }
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
        }
        else {
            // Reset ROI if no markers found
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
        
        // Draw marker on image and publish
        if (drawMarkers)
        {
            cv::aruco::drawDetectedMarkers(imageROI, markerCorners, markerIDs, cv::Scalar(0,0,255));
            markerImagePub.publish(cv_ptr->toImageMsg());
        }
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

void rvec2quat(cv::Vec3d &rvec, tf::Quaternion &quat)
{
    double theta = std::sqrt(std::pow(rvec[0],2) + std::pow(rvec[1],2) + std::pow(rvec[2],2));
    cv::Mat unit = cv::Mat(rvec)/theta;
    
    //std::cout << "rvec: " << rvec << std::endl;
    //std::cout << "theta: " << theta << std::endl;
    //std::cout << "unit: " << unit << std::endl;
    
    quat = tf::Quaternion(std::sin(theta/2)*unit.at<double>(0,0),
                          std::sin(theta/2)*unit.at<double>(1,0),
                          std::sin(theta/2)*unit.at<double>(2,0),
                          std::cos(theta/2));
    //quat.at<double>(0,0) = std::cos(theta/2);
    //quat.at<double>(1,0) = std::sin(theta/2)*unit.at<double>(0,0);
    //quat.at<double>(2,0) = std::sin(theta/2)*unit.at<double>(1,0);
    //quat.at<double>(3,0) = std::sin(theta/2)*unit.at<double>(2,0);
    
    //std::cout << "quat: " << quat << std::endl;
}
