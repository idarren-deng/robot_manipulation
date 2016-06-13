//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

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
#include <opencv2/surface_matching.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/surface_matching/pose_3d.hpp>
#include <opencv2/surface_matching/t_hash_int.hpp>

//pcl
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


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
using namespace ppf_match_3d;

class linemod_detect
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    ros::Subscriber sub_cam_info;
    ros::Publisher pub_pose_viz;
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

    vector<Mat> Rs_,Ts_;
    vector<float> Distances_;
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
    std::string depth_frame_id_;

    std::vector <object_recognition_core::db::ObjData> objs_;

    Mat ppf_train_pc;
    PPF3DDetector ppf_detect;

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_pc_;


public:
        linemod_detect(std::string template_file_name,std::string poses_file_name,std::string mesh_path):
            it(nh),
            sub_color(nh,"/camera/rgb/image_rect_color",1),
            sub_depth(nh,"/camera/depth_registered/image_raw",1),
            depth_frame_id_("camera_rgb_optical_frame"),
            sync(SyncPolicy(10), sub_color, sub_depth),
            px_match_min_(0.25f),
            icp_dist_min_(0.06f),
            th_obj_dist_(0.04f),
            ppf_detect(0.025,0.05),
            model_pc_(new pcl::PointCloud<pcl::PointXYZ>)
        {
            //pub test
            //pub_color_=it.advertise ("/sync_rgb",2);
            //pub_depth=it.advertise ("/sync_depth",2);

            //the intrinsic matrix
            sub_cam_info=nh.subscribe("/camera/depth/camera_info",1,&linemod_detect::read_cam_info,this);

            //ork default param
            threshold=90.0;

            //read the saved linemod detecor
            detector=readLinemod (template_file_name);

            //read the poses of templates
            readLinemodTemplateParams (poses_file_name,
                                       Rs_,
                                       Ts_,
                                       Distances_,
                                       Ks_,
                                       renderer_n_points,
                                       renderer_angle_step,
                                       renderer_radius_min,
                                       renderer_radius_max,
                                       renderer_radius_step,
                                       renderer_width,
                                       renderer_height,
                                       renderer_focal_length_x,
                                       renderer_focal_length_y,
                                       renderer_near,
                                       renderer_far);
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

            ppf_train_pc=loadPLYSimple ("/home/tom/ork_ws/src/ork_tutorials/data/coke_change_origin.ply",1);
            ppf_detect.trainModel (ppf_train_pc);

            //prepare the model
            if(pcl::io::loadPLYFile("/home/tom/ork_ws/src/ork_tutorials/data/coke_change_origin.ply",*model_pc_)==-1)
              {
                cout<<"Load PCD File Failed!"<<endl;
              }

            pub_pose_viz=nh.advertise<sensor_msgs::PointCloud2>("/pose",2);

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
             //namedWindow ("display");

             pci_real_icpin_model->clear();
             pci_real_icpin_ref->clear();

             int count=0;
             BOOST_FOREACH(const linemod::Match& match,matches){
             //if(!matches.empty ())
             //for(int i=0;i<1;++i)
             //{
                 //get the match and display on 2d image window
                //linemod::Match match=matches[i];
                 const std::vector<cv::linemod::Template>& templates = detector->getTemplates(match.class_id, match.template_id);

                 if(count==0)
                 {
                    Mat display=mat_rgb;
                    drawResponse(templates, num_modalities, display,cv::Point(match.x, match.y), detector->getT(0));
                    imshow("display",display);
                    cv::waitKey (1);
                    count++;
                 }

                 //get the pose
                 cv::Matx33d R_match = Rs_[match.template_id].clone();
                 cv::Vec3d T_match = Ts_[match.template_id].clone();
                 cv::Mat K_matrix= Ks_[match.template_id].clone();
                 float D_match = Distances_[match.template_id];

                 //get the point cloud of the rendered object model
                 cv::Mat mask;
                 cv::Rect rect;
                 cv::Matx33d R_temp(R_match.inv());
                 cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));
                 cv::Mat depth_ref_;
                 renderer_iterator_->renderDepthOnly(depth_ref_, mask, rect, -T_match, up);//up?
//                 imshow("depth_ref_",depth_ref_);
//                 cv::waitKey (1);
                 cv::Mat_<cv::Vec3f> depth_real_model_raw;
                 //?K_matrix or K_depth
                 cv::rgbd::depthTo3d(depth_ref_, K_matrix, depth_real_model_raw);

                 //prepare the bounding box for the model and reference point clouds
                 cv::Rect_<int> rect_model(0, 0, depth_real_model_raw.cols, depth_real_model_raw.rows);

                 //prepare the bounding box for the reference point cloud: add the offset
                 cv::Rect_<int> rect_ref(rect_model);
                 rect_ref.x += match.x;
                 rect_ref.y += match.y;

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

                 //transform thr storage format for normal estimation
                 //-------------this way not working right now---------
//                 int num_pts=depth_real_ref.cols*depth_real_ref.rows;
//                 cv::Mat_<cv::Vec3f>::iterator it_real_ref=depth_real_ref.begin ();
//                 Mat ppf_scene_cloud(num_pts,3,CV_32FC1);
//                 for (int i = 0; it_real_ref!=depth_real_ref.end(); i++,it_real_ref++)
//                 {
//                   float* data = (float*)(&ppf_scene_cloud.data[i*ppf_scene_cloud.step[0]]);
//                   data[0]=(*it_real_ref)[0];
//                   data[1]=(*it_real_ref)[1];
//                   data[2]=(*it_real_ref)[2];
//                 }
//                cout<<ppf_scene_cloud<<endl;
//               //compute normal
//               Mat ppf_scene_cloud_normal;
//               double viewpoint[3] = { 0.0, 0.0, 0.0 };
//               computeNormalsPC3d(ppf_scene_cloud, ppf_scene_cloud_normal, 6, false, viewpoint);

               //normal estimation of depth image
               rgbd::RgbdNormals normal_est(depth_real_ref.rows,depth_real_ref.cols,CV_32F,K_depth);
               cv::Mat_<cv::Vec3f> depth_ref_normals;
               normal_est(depth_real_ref,depth_ref_normals);

               //keep those non-nan points
               std::vector<cv::Vec3f> pos_vaild_tmp;
               std::vector<cv::Vec3f> normals_vaild_tmp;
               cv::Mat_<cv::Vec3f>::iterator it_real_ref=depth_real_ref.begin ();
               cv::Mat_<cv::Vec3f>::iterator it_real_ref_normals=depth_ref_normals.begin ();
               for(;it_real_ref!=depth_real_ref.end ();++it_real_ref,++it_real_ref_normals)
               {
                   if(cv::checkRange (*it_real_ref))
                   {
                       pos_vaild_tmp.push_back (*it_real_ref);
                       normals_vaild_tmp.push_back (*it_real_ref_normals);
                   }
               }
               int num_pts=pos_vaild_tmp.size ();
               Mat ppf_scene_cloud_normal(num_pts,6,CV_32FC1);
               for (int i = 0; i<num_pts; i++)
               {
                    float* data = (float*)(&ppf_scene_cloud_normal.data[i*ppf_scene_cloud_normal.step[0]]);
                    data[0]=pos_vaild_tmp[i][0];
                    data[1]=pos_vaild_tmp[i][1];
                    data[2]=pos_vaild_tmp[i][2];
                    data[3]=normals_vaild_tmp[i][0];
                    data[4]=normals_vaild_tmp[i][1];
                    data[5]=normals_vaild_tmp[i][2];
               }
                //cout<<ppf_scene_cloud_normal<<endl;

               //peform ppf detection
               vector<Pose3DPtr> ppf_results;
               ppf_detect.match (ppf_scene_cloud_normal,ppf_results,1.0/1.2,0.05);
               int pause=0;

               Eigen::Matrix4f transform;
                for (int num = 0; num < 2; ++num)
               {
                   for(int row=0;row<4;++row)
                      {
                    for(int col=0;col<4;++col)
                      {
                        transform(row,col)=ppf_results[num]->pose[row*4+col];
                      }
                      }
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output_xyz(new pcl::PointCloud<pcl::PointXYZ>());
                   pcl::transformPointCloud(*model_pc_, *cloud_output_xyz, transform);
                   sensor_msgs::PointCloud2 model_pc;
                   pcl::toROSMsg(*cloud_output_xyz,model_pc);
                   model_pc.header.frame_id="camera_rgb_optical_frame";
                   pub_pose_viz.publish(model_pc);
                }
             }//FOREACH
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
                    float D_tmp;
                    templates["R"]>>R_tmp;
                    Rs.push_back (R_tmp);
                    templates["T"]>>T_tmp;
                    Ts.push_back (T_tmp);
                    templates["K"]>>K_tmp;
                    Ks.push_back (K_tmp);
                    templates["D"]>>D_tmp;
                    Distances.push_back (D_tmp);

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
                            "/home/tom/ork_ws/src/ork_tutorials/data/coke_change_origin.stl");
    ros::spin ();
}
