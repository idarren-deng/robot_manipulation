//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
//#include <cv_bridge/cv_bridge.h>
#include <cv_bridge3/cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/highgui.hpp>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <iostream>

using namespace cv;
using namespace std;

#define normal_windowsize 5
#define plane_threshold 0.02
#define min_table_size 10000

rgbd::RgbdPlane plane_finder;
rgbd::RgbdNormals normal_est;
Mat K_matrix;
vector<float> K_;
bool is_K_read;
ros::Subscriber sub_cam_info;
image_transport::Publisher pub_table_mask;

Mat convertPoint_to_Mat(pcl::PointCloud<pcl::PointXYZ >::Ptr cloud)
{
    Mat OpenCV_Cloud(cloud->height, cloud->width, CV_32FC3);
//#pragma omp parallel for
    if(cloud->isOrganized ())
    {
        for (int i = 0; i < cloud->height; ++i)
        {
            for(int j=0;j<cloud->width;++j)
            {
                pcl::PointXYZ point = cloud->at (j,i);
                OpenCV_Cloud.at<cv::Vec3f>(i,j)[0]=point.x;
                OpenCV_Cloud.at<cv::Vec3f>(i,j)[1]=point.y;
                OpenCV_Cloud.at<cv::Vec3f>(i,j)[2]=point.z;
             }
        }
//code to check if the conversion is correct or not
// method 1
//        int ncols=OpenCV_Cloud.rows*OpenCV_Cloud.cols*OpenCV_Cloud.channels();
//        for(int i=0;i<ncols;i+=3)
//        {
//            cout<<"opencv x = "<<OpenCV_Cloud.at<float>(i)<<"  "<<"PCL point x = "<<cloud->points[int(i/3)].x<<endl;
//            cout<<"opencv y = "<<OpenCV_Cloud.at<float>(i+1)<<"  "<<"PCL point y = "<<cloud->points[int(i/3)].y<<endl;
//            cout<<"opencv z = "<<OpenCV_Cloud.at<float>(i+2)<<"  "<<"PCL point z = "<<cloud->points[int(i/3)].z<<endl;
//            int pause=0;
//        }
// method 2
//        for (int i = 0; i < OpenCV_Cloud.rows; ++i)
//        {
//            for(int j=0;j<OpenCV_Cloud.cols;++j)
//            {
//                cout<<"opencv x = "<<OpenCV_Cloud.at<cv::Vec3f>(i,j)[0]<<"  "<<"PCL point x = "<<cloud->at (j,i).x<<endl;
//                cout<<"opencv y = "<<OpenCV_Cloud.at<cv::Vec3f>(i,j)[1]<<"  "<<"PCL point y = "<<cloud->at (j,i).y<<endl;
//                cout<<"opencv z = "<<OpenCV_Cloud.at<cv::Vec3f>(i,j)[2]<<"  "<<"PCL point z = "<<cloud->at (j,i).z<<endl;
//                int pause=0;
//            }
//        }
        return OpenCV_Cloud;
    }
    else
    {
        ROS_INFO("the PointCloud2 msg is not organised!");
        return OpenCV_Cloud;
    }
}

void read_cam_info(const sensor_msgs::CameraInfoConstPtr& infoMsg)
{
    K_matrix = (cv::Mat_<float>(3, 3) <<
                         infoMsg->K[0], infoMsg->K[1], infoMsg->K[2],
                         infoMsg->K[3], infoMsg->K[4], infoMsg->K[5],
                         infoMsg->K[6], infoMsg->K[7], infoMsg->K[8]);

    Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
                          infoMsg->D[0],
                          infoMsg->D[1],
                          infoMsg->D[2],
                          infoMsg->D[3],
                          infoMsg->D[4]);
    sub_cam_info.shutdown ();
    is_K_read=true;
}

void initiate_plane_finder(double plane_threshold_,int min_table_size_)
{
    plane_finder.setThreshold (plane_threshold_);
    plane_finder.setMinSize (min_table_size_);
    plane_finder.setSensorErrorA (0.0075);
//    K_matrix.at<float>(0,0)=570.342;
//    K_matrix.at<float>(0,1)=0.0;
//    K_matrix.at<float>(0,2)=314.5;
//    K_matrix.at<float>(1,0)=0.0;
//    K_matrix.at<float>(1,1)=570.342;
//    K_matrix.at<float>(1,2)=235.5;
//    K_matrix.at<float>(2,0)=0.0;
//    K_matrix.at<float>(2,1)=0.0;
//    K_matrix.at<float>(2,2)=1.0;
//    K_.resize (9);
//    K_[0]=570.342;
//    K_[1]=0;
//    K_[2]=314.5;
//    K_[3]=0;
//    K_[4]=570.342;
//    K_[5]=235.5;
//    K_[6]=0.0;
//    K_[7]=0.0;
//    K_[8]=1.0;
}

void depthImg_cb(const sensor_msgs::PointCloud2ConstPtr& pointCloud)
{

    if(is_K_read)
        {
        //estimate normals
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Mat cloud_cv,normals;
        pcl::fromROSMsg(*pointCloud,*cloud);
        cloud_cv=convertPoint_to_Mat (cloud);
        MatIterator_<float> it=K_matrix.begin<float>();
        for(;it!=K_matrix.end<float>();it++)
        {
            cout<<" "<<*it<<" "<<endl;
        }
        normal_est=cv::rgbd::RgbdNormals(480,640,CV_32F,K_matrix,5,rgbd::RgbdNormals::RGBD_NORMALS_METHOD_FALS);
        normal_est(cloud_cv,normals);
        //find plane
        Mat table_mask;
        vector<Vec4f> coefficients;
        plane_finder(cloud_cv,normals,table_mask,coefficients);
        //pub_table_mask.publish (table_mask);
        }
//    cv_bridge::CvImagePtr bridgePtr;
//    bridgePtr=cv_bridge::toCvCopy (ros_imgPtr,"32FC1");
//    //cv::circle(bridgePtr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
//    bridgePtr->header.frame_id="/camera_link";
//    pub.publish(bridgePtr->toImageMsg ());
}

int main(int argc,char** argv)
{
  is_K_read=false;
  initiate_plane_finder(plane_threshold,min_table_size);

  ros::init(argc,argv,"board_finder_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  pub_table_mask=it.advertise ("table_mask",1);
  sub_cam_info=nh.subscribe("/camera/depth/camera_info",1,read_cam_info);
  ros::Subscriber sub=nh.subscribe("camera/depth_registered/points",5,depthImg_cb);

  ros::spin();
  return 0;
}
