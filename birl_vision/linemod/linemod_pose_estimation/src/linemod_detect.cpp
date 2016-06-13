//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

//tf
#include <tf/transform_broadcaster.h>

//#include <cv_bridge/cv_bridge.h>
#include <cv_bridge3/cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

//ork
#include "linemod_icp.h"
#include "linemod_pointcloud.h"
#include "db_linemod.h"
#include <object_recognition_renderer/renderer3d.h>
#include <object_recognition_renderer/utils.h>
#include <object_recognition_core/common/pose_result.h>
//#include <object_recognition_core/db/ModelReader.h>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/highgui.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

//boost
#include <boost/foreach.hpp>

#include <math.h>

//time synchronize
#define APPROXIMATE

#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif

#ifdef EXACT
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
#endif
#ifdef APPROXIMATE
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
#endif

//namespace
using namespace cv;
using namespace std;

struct LinemodData{
  LinemodData(
          std::vector<cv::Vec3f> _pts_ref,
          std::vector<cv::Vec3f> _pts_model,
          std::string _match_class,
          const float _match_sim,
          const float _icp_distance,
          const cv::Matx33f _r,
          const cv::Vec3f _t){
    pts_ref = _pts_ref;
    pts_model = _pts_model;
    match_class = _match_class;
    match_sim = _match_sim;
    icp_distance = _icp_distance;
    r = _r;
    t = _t;
    check_done = false;
  }
  std::vector<cv::Vec3f> pts_ref;
  std::vector<cv::Vec3f> pts_model;
  std::string match_class;
  float match_sim;
  float icp_distance;
  cv::Matx33f r;
  cv::Vec3f t;
  bool check_done;
};

class linemod_detect
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    ros::Subscriber sub_cam_info;
    //image_transport::Subscriber sub_color_;
    //image_transport::Subscriber sub_depth;
    //image_transport::Publisher pub_color_;
    //image_transport::Publisher pub_depth;
    message_filters::Subscriber<sensor_msgs::Image> sub_color;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth;
    message_filters::Synchronizer<SyncPolicy> sync;

public:
    Ptr<linemod::Detector> detector;
    float threshold;
    bool is_K_read;
    cv::Vec3f T_ref;

    vector<Mat> Rs_,Ts_;
    vector<float> Distances_;
    vector<float> Obj_origin_dists;
    vector<Mat> Ks_;
    Mat K_depth;
    int renderer_n_points;
    int renderer_angle_step;
    double renderer_radius_min;
    double renderer_radius_max;
    double renderer_radius_step;
    int renderer_width;
    int renderer_height;
    double renderer_focal_length_x;
    double renderer_focal_length_y;
    double renderer_near;
    double renderer_far;
    Renderer3d *renderer_;
    RendererIterator *renderer_iterator_;

    float px_match_min_;
    float icp_dist_min_;
    float th_obj_dist_;

    LinemodPointcloud *pci_real_icpin_ref;
    LinemodPointcloud *pci_real_icpin_model;
    LinemodPointcloud *pci_real_nonICP_model;
    LinemodPointcloud *pci_real_1stICP_model;
    std::string depth_frame_id_;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    //std::vector <object_recognition_core::db::ObjData> objs_;
    std::vector<LinemodData> objs_;//pose result after clustering process
    std::vector<LinemodData> final_poses;//pose result after clustering process

    tf::TransformBroadcaster tf_broadcaster;
public:
        linemod_detect(std::string template_file_name,std::string poses_file_name,std::string mesh_path):
            it(nh),
            sub_color(nh,"/camera/rgb/image_rect_color",1),
            sub_depth(nh,"/camera/depth_registered/image_raw",1),
            depth_frame_id_("camera_rgb_optical_frame"),
            sync(SyncPolicy(10), sub_color, sub_depth),
            px_match_min_(0.25f),
            icp_dist_min_(0.06f),
            th_obj_dist_(0.04f)
        {
            //pub test
            //pub_color_=it.advertise ("/sync_rgb",2);
            //pub_depth=it.advertise ("/sync_depth",2);

            //the intrinsic matrix
            sub_cam_info=nh.subscribe("/camera/depth/camera_info",1,&linemod_detect::read_cam_info,this);

            //ork default param
            threshold=91.0;

            //read the saved linemod detecor
            detector=readLinemod (template_file_name);

            //read the poses of templates
            readLinemodTemplateParams (poses_file_name,Rs_,Ts_,Distances_,Obj_origin_dists,
                                       Ks_,renderer_n_points,renderer_angle_step,
                                       renderer_radius_min,renderer_radius_max,
                                       renderer_radius_step,renderer_width,renderer_height,
                                       renderer_focal_length_x,renderer_focal_length_y,
                                       renderer_near,renderer_far);
            //load the stl model to GL renderer
            renderer_ = new Renderer3d(mesh_path);
            renderer_->set_parameters(renderer_width, renderer_height, renderer_focal_length_x, renderer_focal_length_y, renderer_near, renderer_far);
            renderer_iterator_ = new RendererIterator(renderer_, renderer_n_points);
            renderer_iterator_->angle_step_ = renderer_angle_step;
            renderer_iterator_->radius_min_ = float(renderer_radius_min);
            renderer_iterator_->radius_max_ = float(renderer_radius_max);
            renderer_iterator_->radius_step_ = float(renderer_radius_step);

            pci_real_icpin_model = new LinemodPointcloud(nh, "real_icpin_model", depth_frame_id_);
            pci_real_icpin_ref = new LinemodPointcloud(nh, "real_icpin_ref", depth_frame_id_);
            pci_real_nonICP_model= new LinemodPointcloud(nh, "real_nonICP_model", depth_frame_id_);
            pci_real_1stICP_model= new LinemodPointcloud(nh, "real_1stICP_model", depth_frame_id_);

            icp.setMaximumIterations (25);
            icp.setMaxCorrespondenceDistance (0.05);
            icp.setTransformationEpsilon (1e-3);
            icp.setEuclideanFitnessEpsilon (0.002);

            sync.registerCallback(boost::bind(&linemod_detect::detect_cb,this,_1,_2));
        }

        void detect_cb(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth)
        {

            if(!is_K_read)
                return;

            if(detector->classIds ().empty ())
            {
                ROS_INFO("Linemod detector is empty");
                return;
            }

            cv_bridge::CvImagePtr img_ptr_rgb;
            cv_bridge::CvImagePtr img_ptr_depth;
            std::vector<Mat> sources;

            try{
                 img_ptr_depth = cv_bridge::toCvCopy(*msg_depth);
             }
             catch (cv_bridge::Exception& e)
             {
                 ROS_ERROR("cv_bridge exception:  %s", e.what());
                 return;
             }
             try{
                 img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::BGR8);
             }
             catch (cv_bridge::Exception& e)
             {
                 ROS_ERROR("cv_bridge exception:  %s", e.what());
                 return;
             }

            //publish test
            //pub_color_.publish (img_ptr_rgb->toImageMsg ());
            //pub_depth.publish (img_ptr_depth->toImageMsg ());

             cv::Mat mat_depth;
             if(img_ptr_depth->image.depth ()==CV_32F)
             {
                img_ptr_depth->image.convertTo (mat_depth,CV_16UC1,1000.0);
             }

             //std::cout<<"depth "<<img_ptr_depth->image.at<short>(240,320)<<std::endl;
             cv::Mat mat_rgb;
             if(img_ptr_rgb->image.rows>960)
             {
                 cv::pyrDown (img_ptr_rgb->image.rowRange (0,960),mat_rgb);
             }
             else{
                  mat_rgb = img_ptr_rgb->image;
             }

             //perform the detection
             sources.push_back (mat_rgb);
             sources.push_back (mat_depth);
             std::vector<linemod::Match> matches;
             detector->match (sources,threshold,matches);

             int num_modalities = (int) detector->getModalities().size();

             //convert the depth image to 3d pointcloud
             cv::Mat_<cv::Vec3f> depth_real_ref_raw;
             cv::rgbd::depthTo3d(mat_depth, K_depth, depth_real_ref_raw);

             pci_real_icpin_model->clear();
             pci_real_icpin_ref->clear();
             pci_real_nonICP_model->clear ();
             pci_real_1stICP_model->clear ();

             int count=0;
             //deal with the candidates from the above linemod match
             BOOST_FOREACH(const linemod::Match& match,matches){
                 pci_real_nonICP_model->clear ();
                 pci_real_1stICP_model->clear ();
                 const std::vector<cv::linemod::Template>& templates = detector->getTemplates(match.class_id, match.template_id);

                 //visualize the match on image, only display the first result
                 if(count==0)
                 {
                    Mat display=mat_rgb;
                    drawResponse(templates, num_modalities, display,cv::Point(match.x, match.y), detector->getT(0));
                    imshow("display",display);
                    cv::waitKey (1);
                    count++;
                 }

                 //get the pose
                 cv::Matx33d R_match = Rs_[match.template_id].clone();// rotation of the object w.r.t to the view point
                 cv::Vec3d T_match = Ts_[match.template_id].clone();//the translation of the camera with respect to the current view point
                 cv::Mat K_matrix= Ks_[match.template_id].clone();
                 float D_match = Distances_[match.template_id];//the distance from the center of object surface to the camera origin

                 //get the point cloud of the rendered object model
                 cv::Mat mask;
                 cv::Rect rect;
                 cv::Matx33d R_temp(R_match.inv());//rotation of the viewpoint w.r.t the object
                 cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));//the negative direction of y axis of viewpoint w.r.t object frame
                 cv::Mat depth_ref_;
                 renderer_iterator_->renderDepthOnly(depth_ref_, mask, rect, -T_match, up);//up?

                 //imshow("depth_ref_",mask);
                 cv::waitKey (1);
                 cv::Mat_<cv::Vec3f> depth_real_model_raw;
                 cv::rgbd::depthTo3d(depth_ref_, K_matrix, depth_real_model_raw);//unit mm to m

                 //prepare the bounding box for the model and reference point clouds
                 cv::Rect_<int> rect_model(0, 0, depth_real_model_raw.cols, depth_real_model_raw.rows);

                 //prepare the bounding box for the reference point cloud: add the offset
                 cv::Rect_<int> rect_ref(rect_model);
                 rect_ref.x += match.x;
                 rect_ref.y += match.y;

                 //compute the intersection between template and real scene image
                 rect_ref = rect_ref & cv::Rect(0, 0, depth_real_ref_raw.cols, depth_real_ref_raw.rows);
                 if ((rect_ref.width < 5) || (rect_ref.height < 5))
                   continue;
                 //adjust both rectangles to be equal to the smallest among them
                 if (rect_ref.width > rect_model.width)
                   rect_ref.width = rect_model.width;
                 if (rect_ref.height > rect_model.height)
                   rect_ref.height = rect_model.height;
                 if (rect_model.width > rect_ref.width)
                   rect_model.width = rect_ref.width;
                 if (rect_model.height > rect_ref.height)
                   rect_model.height = rect_ref.height;

                 //prepare the reference data: from the sensor : crop images
                 cv::Mat_<cv::Vec3f> depth_real_ref = depth_real_ref_raw(rect_ref);
                 //prepare the model data: from the match
                 cv::Mat_<cv::Vec3f> depth_real_model = depth_real_model_raw(rect_model);

                 //initialize the translation based on reference data
                 cv::Vec3f T_crop = depth_real_ref(depth_real_ref.rows / 2.0f, depth_real_ref.cols / 2.0f);
                 T_ref=T_crop;
               //first filter step
                 //reject if the depth of the templates exceed the ref depth to a certain extent
                 float center_depth_model=(depth_real_model(depth_real_model.rows/2,depth_real_model.cols/2))[2];
                 float center_depth_ref=(depth_real_ref(depth_real_ref.rows / 2.0f, depth_real_ref.cols / 2.0f))[2];
                 float depth_th=center_depth_ref*0.1;
                 if(fabs(center_depth_model-center_depth_ref)>depth_th)
                     continue;
//              //second filter step
//                 //using depth to filter scene points: only those points with depth within a range will survive
//                    //calculate Z range
//                 float Z_th=T_crop(2);
//                 cv::Mat_<cv::Vec3f>::iterator it_real_model=depth_real_model.begin ();
//                 float Z_min=10000;
//                 float Z_max=0.0;
//                 float Z_range=0.0;
//                 for(;it_real_model!=depth_real_model.end ();++it_real_model)
//                 {
//                     if(!cv::checkRange (*it_real_model))
//                         continue;
//                     //cv::Vec3f tmp=*it_real_model;
//                     if((*it_real_model)[2]<Z_min)
//                         Z_min=(*it_real_model)[2];
//                     if((*it_real_model)[2]>Z_max)
//                        Z_max=(*it_real_model)[2];
//                 }
//                 Z_range=Z_max-Z_min;
//                    //start filtering
//                 cv::Mat_<cv::Vec3f>::iterator it_real_ref= depth_real_ref.begin ();
//                 cv::Mat_<cv::Vec3f> depth_real_ref_filter;
//                 for(;it_real_ref!=depth_real_ref.end ();++it_real_ref)
//                 {
//                     cv::Vec3f ref_tmp=*it_real_ref;
//                     if(cv::checkRange (ref_tmp))
//                     {
//                         if(fabs(ref_tmp[2]-Z_th)<Z_range)
//                             depth_real_ref_filter.push_back(ref_tmp);
//                     }

//                 }

                 //add the object's depth
                 T_crop(2) += D_match;
                 if (!cv::checkRange(T_crop))
                   continue;
                 cv::Vec3f T_real_icp(T_crop);

                 //initialize the rotation based on model data
                 //R_match orientation of obj w.r.t view frame(camera frame)
                 if (!cv::checkRange(R_match))
                   continue;
                 cv::Matx33f R_real_icp(R_match);

                 //get the point clouds (for both reference and model)
                 std::vector<cv::Vec3f> pts_real_model_temp;
                 std::vector<cv::Vec3f> pts_real_ref_temp;
                 float px_ratio_missing = matToVec(depth_real_ref, depth_real_model, pts_real_ref_temp, pts_real_model_temp);

                 //before using ICP
//                 pci_real_nonICP_model->fill (pts_real_model_temp,cv::Vec3b(255,0,0));
//                 pci_real_nonICP_model->publish ();

                //third filter step
                 //reject if the points of model are too less regard to scene points
                 if (px_ratio_missing > (1.0f-px_match_min_))
                   continue;

                 //add the Translation vector according to the position of point cloud of interest(ref points)
                 Matx33f R_tmp=Matx<float,3,3>(1.0,0.0,0.0,
                                               0.0,1.0,0.0,
                                               0.0,0.0,1.0);
                 cv::Vec3f T_vect;
                 //Origin is set in the geometry center
                 cv::Vec3f T_real_model=depth_real_model(depth_real_model.rows/2,depth_real_model.cols/2);
                 //Origin not in the center
                 T_vect[0]=T_ref[0]-T_real_model[0];
                 T_vect[1]=T_ref[1]-T_real_model[1];
                 T_vect[2]=T_ref[2]-T_real_model[2];
                 transformPoints(pts_real_model_temp, pts_real_model_temp, R_tmp, T_vect);
                 //update the position
                 T_real_model[2]+=D_match; //attention! if Origin is in the center,please uncomment this
                 T_real_model+=T_vect;
                 T_real_icp=T_real_model;


                 //PCL ICP refine step
                 //transform the format
                 pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pc(new pcl::PointCloud<pcl::PointXYZ>);
                 pcl::PointCloud<pcl::PointXYZ>::Ptr model_pc(new pcl::PointCloud<pcl::PointXYZ>);
                 ref_pc->points.resize (pts_real_ref_temp.size());
                 model_pc->points.resize(pts_real_model_temp.size ());

                 for(int i=0;i<pts_real_ref_temp.size();++i)
                 {
                     ref_pc->points[i].x=pts_real_ref_temp[i][0];
                     ref_pc->points[i].y=pts_real_ref_temp[i][1];
                     ref_pc->points[i].z=pts_real_ref_temp[i][2];
                 }

                 for(int i=0;i<pts_real_model_temp.size();++i)
                 {
                     model_pc->points[i].x=pts_real_model_temp[i][0];
                     model_pc->points[i].y=pts_real_model_temp[i][1];
                     model_pc->points[i].z=pts_real_model_temp[i][2];
                 }

                 icp.setInputSource (model_pc);
                 icp.setInputTarget (ref_pc);
                 icp.align (*model_pc);
                 float icp_distance_=icp.getFitnessScore ();


                 if (icp.hasConverged ())
                 {
                   Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation ().cast<double>();
                   for(int i=0;i<3;++i)
                   {
                       for(int j=0;j<3;++j)
                       {
                           R_tmp(i,j)=transformation_matrix(i,j);
                       }
                   }
                   T_vect[0]=transformation_matrix(0,3);
                   T_vect[1]=transformation_matrix(1,3);
                   T_vect[2]=transformation_matrix(2,3);

                   //update the pointcloud
                   transformPoints(pts_real_model_temp, pts_real_model_temp, R_tmp, T_vect);                   
                   //update the translation vector
                   cv::Vec3f T_tmp;
                   T_tmp = R_tmp * T_real_icp;
                   cv::add(T_vect, T_tmp, T_real_icp);
                   //update the rotation matrix
                   R_real_icp = R_tmp * R_real_icp;
                   pci_real_1stICP_model->fill (pts_real_model_temp,cv::Vec3b(0,255,0));
                   pci_real_1stICP_model->publish ();
                 }
                 else
                 {
                    continue;
                 }
                 objs_.push_back(LinemodData(pts_real_ref_temp, pts_real_model_temp, match.class_id, match.similarity, icp_distance_, R_real_icp, T_real_icp));
             }//FOREACH

             //local non-maxima supression to find the best match at each position
             int count_pass = 0;
             std::vector <LinemodData>::iterator it_o = objs_.begin();
             for (; it_o != objs_.end(); ++it_o)
              {
                 if (!it_o->check_done)
               {
                 //initialize the object to publish
                 LinemodData *o_match = &(*it_o);
                 //static_cast used as force convertion
                 int size_th = static_cast<int>((float)o_match->pts_model.size()*0.85);
                 std::vector <LinemodData>::iterator it_o2 = it_o;
                 ++it_o2;
                 //find the best object match among near objects
                 for (; it_o2 != objs_.end(); ++it_o2)
                   {
                     if (!it_o2->check_done)
                     if (cv::norm(o_match->t, it_o2->t) < th_obj_dist_)
                     {
                       it_o2->check_done = true;
                       if ((it_o2->pts_model.size() > size_th) && (it_o2->icp_distance < o_match->icp_distance))
                         o_match = &(*it_o2);
                     }
                   }

                 //perform the final precise icp

//                 pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pc(new pcl::PointCloud<pcl::PointXYZ>);
//                 pcl::PointCloud<pcl::PointXYZ>::Ptr model_pc(new pcl::PointCloud<pcl::PointXYZ>);
//                 cv::Vec3f T_vect;
//                 Matx33f R_tmp;
//                 ref_pc->points.resize (o_match->pts_ref.size ());
//                 model_pc->points.resize(o_match->pts_model.size());

//                 for(int i=0;i<o_match->pts_ref.size();++i)
//                 {
//                     ref_pc->points[i].x=o_match->pts_ref[i][0];
//                     ref_pc->points[i].y=o_match->pts_ref[i][1];
//                     ref_pc->points[i].z=o_match->pts_ref[i][2];
//                 }

//                 for(int i=0;i<o_match->pts_model.size();++i)
//                 {
//                     model_pc->points[i].x=o_match->pts_model[i][0];
//                     model_pc->points[i].y=o_match->pts_model[i][1];
//                     model_pc->points[i].z=o_match->pts_model[i][2];
//                 }

//                 pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//                 icp.setMaximumIterations (50);
//                 icp.setMaxCorrespondenceDistance (0.05);
//                 icp.setInputSource (model_pc);
//                 icp.setInputTarget (ref_pc);
//                 icp.setTransformationEpsilon (1e-4);
//                 icp.setEuclideanFitnessEpsilon (0.004);
//                 icp.align (*model_pc);
//                 float icp_distance_=icp.getFitnessScore ();


//                 if (icp.hasConverged ())
//                 {
//                   Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation ().cast<double>();
//                   for(int i=0;i<3;++i)
//                   {
//                       for(int j=0;j<3;++j)
//                       {
//                           R_tmp(i,j)=transformation_matrix(i,j);
//                       }
//                   }
//                   T_vect[0]=transformation_matrix(0,3);
//                   T_vect[1]=transformation_matrix(1,3);
//                   T_vect[2]=transformation_matrix(2,3);

//                   //update the pointcloud
//                   transformPoints(o_match->pts_model, o_match->pts_model, R_tmp, T_vect);
//                   //update the translation vector
//                   o_match->t = R_tmp * o_match->t;
//                   cv::add(T_vect, o_match->t, o_match->t);
//                   //update the rotation matrix
//                   o_match->r = R_tmp * o_match->r;

                 //add points to the clouds
                 pci_real_icpin_model->fill(o_match->pts_model, cv::Vec3b(0,255,0));
                 pci_real_icpin_ref->fill(o_match->pts_ref, cv::Vec3b(0,0,255));
                 pci_real_icpin_model->publish();
                 pci_real_icpin_ref->publish();

                 final_poses.push_back (*o_match);
                 ++count_pass;
                }
              }//non-maximum supression finished

             // broadcast TFs
             for(std::vector<LinemodData>::iterator it_pose=final_poses.begin ();it_pose!=final_poses.end();++it_pose)
             {
                 cv::Vec3f position=it_pose->t;
                 cv::Matx33f orientation=it_pose->r;
                 tf::Transform transform;
                 tf::Matrix3x3 orientation3x3_tf;
                 transform.setOrigin (tf::Vector3(position[0],position[1],position[2]));
                 orientation3x3_tf.setValue (orientation(0,0),orientation(0,1),orientation(0,2),
                                          orientation(1,0),orientation(1,1),orientation(1,2),
                                          orientation(2,0),orientation(2,1),orientation(2,2));
                 tf::Quaternion orientationQua_tf;
                 orientation3x3_tf.getRotation (orientationQua_tf);
                 transform.setRotation (orientationQua_tf);
                 tf_broadcaster.sendTransform (tf::StampedTransform(transform,ros::Time::now(),"camera_rgb_optical_frame","coke_frame"));

                 //debug info
//                 cout<<orientation<<endl;
//                 cout<<orientation(0,0)<<endl;
//                 cout<<orientation(0,1)<<endl;
//                 cout<<orientation(0,2)<<endl;
//                 int pause=0;
             }
             std::cout << "rough matches:" << matches.size()<<" / "<<"condition filter: "<<objs_.size() << " / " <<"after clustering:"<< count_pass<<std::endl;
             objs_.clear ();
             final_poses.clear ();
             //publish the point clouds


        }

        static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
        {
          cv::Ptr<cv::linemod::Detector> detector = cv::makePtr<cv::linemod::Detector>();
          cv::FileStorage fs(filename, cv::FileStorage::READ);
          detector->read(fs.root());

          cv::FileNode fn = fs["classes"];
          for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
            detector->readClass(*i);

          return detector;
        }

        void drawResponse(const std::vector<cv::linemod::Template>& templates, int num_modalities, cv::Mat& dst, cv::Point offset,
                     int T)
        {
          static const cv::Scalar COLORS[5] =
          { CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 255, 0), CV_RGB(255, 140, 0), CV_RGB(255, 0, 0) };
          if (dst.channels() == 1)
            cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);

          cv::circle(dst, cv::Point(offset.x + 20, offset.y + 20), T / 2, COLORS[4]);
          if (num_modalities > 5)
            num_modalities = 5;
          for (int m = 0; m < num_modalities; ++m)
          {
        // NOTE: Original demo recalculated max response for each feature in the TxT
        // box around it and chose the display color based on that response. Here
        // the display color just depends on the modality.
            cv::Scalar color = COLORS[m];

            for (int i = 0; i < (int) templates[m].features.size(); ++i)
            {
              cv::linemod::Feature f = templates[m].features[i];
              cv::Point pt(f.x + offset.x, f.y + offset.y);
              cv::circle(dst, pt, T / 2, color);
            }
          }
        }

         void readLinemodTemplateParams(const std::string fileName,
                                        std::vector<cv::Mat>& Rs,
                                        std::vector<cv::Mat>& Ts,
                                        std::vector<float>& Distances,
                                        std::vector<float>& Obj_origin_dists,
                                        std::vector<cv::Mat>& Ks,
                                        int& renderer_n_points,
                                        int& renderer_angle_step,
                                        double& renderer_radius_min,
                                        double& renderer_radius_max,
                                        double& renderer_radius_step,
                                        int& renderer_width,
                                        int& renderer_height,
                                        double& renderer_focal_length_x,
                                        double& renderer_focal_length_y,
                                        double& renderer_near,
                                        double& renderer_far)
        {
            FileStorage fs(fileName,FileStorage::READ);

            for(int i=0;;++i)
            {
                std::stringstream ss;
                std::string s;
                s="Template ";
                ss<<i;
                s+=ss.str ();
                FileNode templates = fs[s];
                if(!templates.empty ())
                {
                    Mat R_tmp,T_tmp,K_tmp;
                    float D_tmp,obj_dist_tmp;
                    templates["R"]>>R_tmp;
                    Rs.push_back (R_tmp);
                    templates["T"]>>T_tmp;
                    Ts.push_back (T_tmp);
                    templates["K"]>>K_tmp;
                    Ks.push_back (K_tmp);
                    templates["D"]>>D_tmp;
                    Distances.push_back (D_tmp);
                    templates["Ori_dist"] >>obj_dist_tmp;
                    Obj_origin_dists.push_back (obj_dist_tmp);

                }
                else
                {
                    //fs["K Intrinsic Matrix"]>>K_matrix;
                    //std::cout<<K_matrix<<std::endl;
                    fs["renderer_n_points"]>>renderer_n_points;
                    fs["renderer_angle_step"]>>renderer_angle_step;
                    fs["renderer_radius_min"]>>renderer_radius_min;
                    fs["renderer_radius_max"]>>renderer_radius_max;
                    fs["renderer_radius_step"]>>renderer_radius_step;
                    fs["renderer_width"]>>renderer_width;
                    fs["renderer_height"]>>renderer_height;
                    fs["renderer_focal_length_x"]>>renderer_focal_length_x;
                    fs["renderer_focal_length_y"]>>renderer_focal_length_y;
                    fs["renderer_near"]>>renderer_near;
                    fs["renderer_far"]>>renderer_far;
                    break;
                }
            }

            fs.release ();
        }

         void read_cam_info(const sensor_msgs::CameraInfoConstPtr& infoMsg)
         {
             K_depth = (cv::Mat_<float>(3, 3) <<
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
};

int main(int argc,char** argv)
{
    ros::init (argc,argv,"linemod_detect");
    linemod_detect detector("/home/tom/catkin_ws/devel/lib/model_board_finder/coke_linemod_template_xtion.yml",
                            "/home/tom/catkin_ws/devel/lib/model_board_finder/template_params_xtion.yml",
                            "/home/tom/ork_ws/src/ork_tutorials/data/box.stl");
    ros::spin ();
}
