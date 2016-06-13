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

#include <object_recognition_renderer/renderer3d.h>
#include <object_recognition_renderer/utils.h>

// Functions to store detector and templates in single XML/YAML file
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

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector->write(fs);

  std::vector<cv::String> ids = detector->classIds();
  fs << "classes" << "[";
  for (int i = 0; i < (int)ids.size(); ++i)
  {
    fs << "{";
    detector->writeClass(ids[i], fs);
    fs << "}"; // current class
  }
  fs << "]"; // classes
}

static void writeLinemodTemplateParams(std::string fileName,
                                       std::vector<cv::Mat>& Rs,
                                       std::vector<cv::Mat>& Ts,
                                       std::vector<float>& distances,
                                       std::vector<float>& obj_origin_dists,
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
    cv::FileStorage fs(fileName,cv::FileStorage::WRITE);
    int num_templates=Rs.size ();
    for(int i=0;i<num_templates;++i)
    {
        std::stringstream ss;
        std::string a;
        ss<<i;
        a="Template ";
        a+=ss.str ();
        fs<<a<<"{";
        fs<<"ID"<<i;
        fs<<"R"<<Rs[i];
        fs<<"T"<<Ts[i];
        fs<<"K"<<Ks[i];
        fs<<"D"<<distances[i];
        fs<<"Ori_dist"<<obj_origin_dists[i];
        fs<<"}";
    }
    //fs<<"K Intrinsic Matrix"<<cv::Mat(K_matrix);
    fs<<"renderer_n_points"<<renderer_n_points;
    fs<<"renderer_angle_step"<<renderer_angle_step;
    fs<<"renderer_radius_min"<<renderer_radius_min;
    fs<<"renderer_radius_max"<<renderer_radius_max;
    fs<<"renderer_radius_step"<<renderer_radius_step;
    fs<<"renderer_width"<<renderer_width;
    fs<<"renderer_height"<<renderer_height;
    fs<<"renderer_focal_length_x"<<renderer_focal_length_x;
    fs<<"renderer_focal_length_y"<<renderer_focal_length_y;
    fs<<"renderer_near"<<renderer_near;
    fs<<"renderer_far"<<renderer_far;
    fs.release ();
}

int main(int argc,char** argv)
{
    cv::Ptr<cv::linemod::Detector> detector_ = cv::linemod::getDefaultLINEMOD();
    int renderer_n_points_ = 150;
    int renderer_angle_step_ = 10;
    double renderer_radius_min_ = 0.6;
    double renderer_radius_max_ = 1.1;
    double renderer_radius_step_ = 0.4;
    int renderer_width_ = 640;
    int renderer_height_ = 480;
    double renderer_near_ = 0.1;
    double renderer_far_ = 1000.0;
    double renderer_focal_length_x_ = atof(argv[1]);//Kinect ;//xtion 570.342;
    double renderer_focal_length_y_ = atof(argv[2]);//Kinect //xtion 570.342;
    std::string stl_file=argv[3];
    std::string template_output_path=argv[4];
    std::string renderer_params_output_path=argv[5];

    Renderer3d render=Renderer3d(stl_file);
    render.set_parameters (renderer_width_, renderer_height_, renderer_focal_length_x_,
                           renderer_focal_length_y_, renderer_near_, renderer_far_);
    RendererIterator renderer_iterator=RendererIterator(&render,renderer_n_points_);
    renderer_iterator.angle_step_ = renderer_angle_step_;
    renderer_iterator.radius_min_ = float(renderer_radius_min_);
    renderer_iterator.radius_max_ = float(renderer_radius_max_);
    renderer_iterator.radius_step_ = float(renderer_radius_step_);

    cv::Mat image, depth, mask;
    cv::Matx33d R;
    cv::Vec3d T;
    cv::Matx33f K;
    std::vector<cv::Mat> Rs_;
    std::vector<cv::Mat> Ts_;
    std::vector<float> distances_;
    std::vector<float> Origin_dists_;
    std::vector<cv::Mat> Ks_;

    for (size_t i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator)
    {
      std::stringstream status;
      status << "Loading images " << (i+1) << "/"
          << renderer_iterator.n_templates();
      std::cout << status.str();

      cv::Rect rect;
      renderer_iterator.render(image, depth, mask, rect);

      R = renderer_iterator.R_obj();
      T = renderer_iterator.T();
      //D_obj distance from camera to object origin
      float distance = renderer_iterator.D_obj() - float(depth.at<ushort>(depth.rows/2.0f, depth.cols/2.0f)/1000.0f);
      float obj_origin_dist=renderer_iterator.D_obj();
      K = cv::Matx33f(float(renderer_focal_length_x_), 0.0f, float(rect.width)/2.0f, 0.0f, float(renderer_focal_length_y_), float(rect.height)/2.0f, 0.0f, 0.0f, 1.0f);

      std::vector<cv::Mat> sources(2);
      sources[0] = image;
      sources[1] = depth;


      // Display the rendered image
      if (true)
      {
        cv::namedWindow("Rendering RGB");
        cv::namedWindow("Rendering Depth");
        cv::namedWindow("Rendering Mask");
        if (!image.empty()) {
          cv::imshow("Rendering RGB", image);
          cv::imshow("Rendering Depth", depth);
          cv::imshow("Rendering Mask", mask);
          cv::waitKey(1);
        }
      }


      int template_in = detector_->addTemplate(sources, "coke", mask);
      if (template_in == -1)
      {
        // Delete the status
        for (size_t j = 0; j < status.str().size(); ++j)
          std::cout << '\b';
        continue;
      }

      // Also store the pose of each template
      Rs_.push_back(cv::Mat(R));
      Ts_.push_back(cv::Mat(T));
      distances_.push_back(distance);
      Ks_.push_back(cv::Mat(K));
      Origin_dists_.push_back (obj_origin_dist);

      // Delete the status
      for (size_t j = 0; j < status.str().size(); ++j)
        std::cout << '\b';
    }

    writeLinemod (detector_,template_output_path);
    writeLinemodTemplateParams (renderer_params_output_path,
                                Rs_,
                                Ts_,
                                distances_,
                                Origin_dists_,
                                Ks_,
                                renderer_n_points_,
                                renderer_angle_step_,
                                renderer_radius_min_,
                                renderer_radius_max_,
                                renderer_radius_step_,
                                renderer_width_,
                                renderer_height_,
                                renderer_focal_length_x_,
                                renderer_focal_length_y_,
                                renderer_near_,
                                renderer_far_);

      return 0;

}
