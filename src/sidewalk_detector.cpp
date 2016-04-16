#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/io/pcd_io.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include <fenv.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


ros::Publisher pubImage, pubPointsIn, pubPointsOut;
int pointTopicInd, imgTopicInd;
static const std::string OPENCV_WINDOW = "Image window";
int dcamWidth, dcamHeight, ccamWidth, ccamHeight;
float distThresh;

void findBestPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inCloud, pcl::ModelCoefficients::Ptr &bestCoeffs, pcl::PointIndices::Ptr &bestInliers, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &insideCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outsideCloud) {
  
  
  pcl::PCDWriter writer;
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());//, bestCoefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); //, bestInliers (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (distThresh);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //variable to track planes with minimum angle
  float minAngle = 180;
  
  
  int i = 0, nr_points = (int) inCloud->points.size ();
  // While 10% of the original cloud is still there
  while (inCloud->points.size () > 0.1 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (inCloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (inCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    //writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    inCloud.swap (cloud_f);
    i++;

    /* calculate the angle between the normal to the plane and y-axis,
       We want the planes represneting the groud so the angle between the normal to the plane
       and the y-axis should be the least */
    std::vector<float> normal;
    for(int x=0; x<3; x++){
      normal.push_back(coefficients->values[x]);
    }
    std::vector<float> y_axis;
    y_axis.push_back(0);
    y_axis.push_back(1);
    y_axis.push_back(0);

    float dot = normal[0]*y_axis[0] + normal[1]*y_axis[1] + normal[2]*y_axis[2];
    float lenSq1 =  normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2];
    float lenSq2 = y_axis[0]*y_axis[0] + y_axis[1]*y_axis[1] + y_axis[2]*y_axis[2];

    float angle =  acos(dot/sqrt(lenSq1 * lenSq2));
    angle = angle * 180 / M_PI;
    angle = fmod(180, angle);
    //cout << " angle between plane and y is " << angle << "\n";
    if(angle < minAngle) {
      minAngle = angle;
      bestInliers = inliers;
      bestCoeffs = coefficients;
    }

    
  }
  

}

void fillImage(cv::Mat &im_out, cv_bridge::CvImagePtr cv_ptr, int fillx, int filly) {
  cv::Vec3b color;
  color[0] = 0;
  color[1] = 0;
  color[2] = 255;
  for(int i=0; i < ccamHeight; i++) {
    int left = -1;
    int right = -1;

    for(int j=0; j < ccamWidth; j++) {
      if(left < 0 && im_out.at<uchar>(cv::Point(j,i)) > 10){
	left = j;
      }
      else if(left >= 0 && im_out.at<uchar>(cv::Point(j,i)) > 10){
	right = j;
      }
    }
    if(left >= 0 && right > left){
      for(int y=left; y < right; y++){
	//cout << " (" << i << ", " << y << ") ";
	im_out.at<uchar>(cv::Point(y,i)) = 255;
	//cv_ptr->image.at<cv::Vec3b>(cv::Point(y,i)) = color;
      }
    }
  }

  for(int i=0; i < ccamWidth; i++) {
    int left = -1;
    int right = -1;

    for(int j=0; j < ccamHeight; j++) {
      if(left < 0 && im_out.at<uchar>(cv::Point(i,j)) > 10){
	left = j;
      }
      else if(left >= 0 && im_out.at<uchar>(cv::Point(i,j)) > 10){
	right = j;
      }
    }
    if(left >= 0 && right > left){
      for(int y=left; y < right; y++){
	//cout << " (" << i << ", " << y << ") ";
	im_out.at<uchar>(cv::Point(i,y)) = 255;
	//cv_ptr->image.at<cv::Vec3b>(cv::Point(i,y)) = color;
      }
    }
  }

  cv::Mat im_floodfill = im_out.clone();
  cv::floodFill(im_floodfill, cv::Point(fillx,filly), cv::Scalar(255));
     
  // Invert floodfilled image
  cv::Mat im_floodfill_inv;
  cv::bitwise_not(im_floodfill, im_floodfill_inv);
     
  // Combine the two images to get the foreground.
  cv::Mat im_out2 = (im_floodfill | im_floodfill_inv);

  for(int i=0; i < ccamHeight; i++) {
    for(int j=0; j < ccamWidth; j++) {
      if(im_out.at<uchar>(cv::Point(j,i)) > 10){
	cv_ptr->image.at<cv::Vec3b>(cv::Point(j,i)) = color;
      }
    }
    }
 }


void callback(const sensor_msgs::PointCloud2ConstPtr& input, const sensor_msgs::ImageConstPtr& imageMsg) {

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  

  pcl::PointCloud<pcl::PointXYZRGB> cloudxx;
  pcl::fromROSMsg(*input,cloudxx);  // Convert ros msg cloud to pcl cloud
 
  pcl::PointCloud<pcl::PointXYZRGB> cloud_copy;
  pcl::copyPointCloud(cloudxx, cloud_copy);


  /* process the cloud to find the best matchind plane for the sidewalk*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr  (new pcl::PointCloud<pcl::PointXYZRGB>(cloud_copy));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudxxPtr  (new pcl::PointCloud<pcl::PointXYZRGB>(cloudxx));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPtr  (new pcl::PointCloud<pcl::PointXYZRGB>(cloudxx));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloudPtr  (new pcl::PointCloud<pcl::PointXYZRGB>(cloudxx));
  pcl::ModelCoefficients::Ptr bestCoeffs (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr bestInliers (new pcl::PointIndices ());
  
  findBestPlane(cloudPtr, bestCoeffs, bestInliers, inCloudPtr, outCloudPtr);
  
  /* the bestInliers are for a point cloud with some of the planes removed,
     The followin code is used to get the inlier indices of the plane in
     the original point cloud */
  Eigen::Vector4f coefficients = Eigen::Vector4f(bestCoeffs->values[0], bestCoeffs->values[1], bestCoeffs->values[2], bestCoeffs->values[3]);
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr samplePlane (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloudxxPtr));
  std::vector<int> inliers; 
  samplePlane->selectWithinDistance (coefficients, distThresh, inliers);

  /* Now saperate the points inside the plane and points outside the plane to publish */
  boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (inliers)); 
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // Extract the inliers
  extract.setInputCloud (cloudxxPtr);
  extract.setIndices (indicesptr);
  extract.setNegative (false);
  extract.filter (*inCloudPtr);

  extract.setNegative (true);
  extract.filter (*outCloudPtr);
  
  /* Project the points given by the inliers into the color camera frame by
     using the color camera projection matrix */
  cv::Mat projMat = (cv::Mat_<double>(3,4) << 624.9082641601562, 0.0, 327.3291931152344, 0.0, 0.0,
	      624.9082641601562, 242.2230987548828, 0.0, 0.0, 0.0, 1.0, 0.0);
  cv::Mat maskMat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1, cv::Scalar(0));

  int fillx, filly;
  for(int i=0; i<inliers.size(); i++) {
    int ind;
    ind = inliers[i];
    cv::Mat worldPt = (cv::Mat_<double>(4,1) << cloudxxPtr->points[ind].x, cloudxxPtr->points[ind].y, cloudxxPtr->points[ind].z , 1); 
    cv::Mat imagePt = (cv::Mat_<double>(3,1));
    imagePt = projMat*worldPt;

    int x,y;
    x = imagePt.at<double>(0)/imagePt.at<double>(2);
    y = imagePt.at<double>(1)/imagePt.at<double>(2);
    if(x >= ccamWidth)
      x = ccamWidth-1;
    if(y >= ccamHeight)
      y = ccamHeight-1;
    if(x < 0)
      x = 0;
    if(y < 0)
      y = 0;

    maskMat.at<uchar>(cv::Point(x,y)) = 255;
    fillx = x;
    filly = y;
  }

  cv::Mat im_out = maskMat;

  //fill in the pixels inside the sidewalk using the mask image form pointcloud
  fillImage(im_out, cv_ptr, fillx, filly);

  // Publish stuff
  sensor_msgs::PointCloud2 inCloudMsg;
  pcl::toROSMsg(*inCloudPtr, inCloudMsg);

  sensor_msgs::PointCloud2 outCloudMsg;
  pcl::toROSMsg(*outCloudPtr, outCloudMsg);

  pubImage.publish(cv_ptr->toImageMsg());
  pubPointsIn.publish(inCloudMsg);
  pubPointsOut.publish(outCloudMsg);
}

int main(int argc, char **argv)
{
  dcamWidth = 480;
  dcamHeight = 360;
  ccamWidth = 640;
  ccamHeight = 480;
  distThresh = 0.1;
  
    ros::init(argc, argv, "sidewalk_detector");
    ros::NodeHandle n;

    cv::namedWindow(OPENCV_WINDOW);
    message_filters::Subscriber<sensor_msgs::PointCloud2> point_sub(n, "/camera/depth/points", 1000);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/camera/color/image_raw", 1000);
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(point_sub, image_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    pubPointsIn = n.advertise<sensor_msgs::PointCloud2> ("sidewalk_detector/points_in", 100);
    pubPointsOut = n.advertise<sensor_msgs::PointCloud2> ("sidewalk_detector/points_out", 100);
    pubImage = n.advertise<sensor_msgs::Image> ("sidewalk_detector/image_raw", 100);
    ros::spin();

    return 0;
}
