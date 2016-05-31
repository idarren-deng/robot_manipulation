/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software 
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, t_config_DI
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik_baxter/GetConstrainedPositionIK.h>
#include <sensor_msgs/JointState.h>

class BaxterTracIKServer {
private:
    std::string _side;
    double _timeout;
    std::string _urdf_param;
    TRAC_IK::TRAC_IK *_tracik_solver;
    KDL::Chain _chain;
    KDL::JntArray *_nominal;
    ros::ServiceServer _service;
    bool _ready;

public:
    BaxterTracIKServer(std::string side, double timeout, std::string urdf_param) {
        double eps = 1e-5;
        this->_ready = false;
        this->_side = side;
        this->_urdf_param = urdf_param;
        this->_timeout = timeout;
        this->_tracik_solver = new TRAC_IK::TRAC_IK("base", side + "_gripper", urdf_param, timeout, eps);

        KDL::JntArray ll, ul; //lower joint limits, upper joint limits

        if(!(this->_tracik_solver->getKDLChain(this->_chain))) {
          ROS_ERROR("There was no valid KDL chain found");
          exit(EXIT_FAILURE);
        }

        if(!(this->_tracik_solver->getKDLLimits(ll,ul))) {
          ROS_ERROR("There were no valid KDL joint limits found");
          exit(EXIT_FAILURE);
        }

        if(!(this->_chain.getNrOfJoints() == ll.data.size())
           || !(this->_chain.getNrOfJoints() == ul.data.size())) {
            ROS_ERROR("Inconsistent joint limits found");
            exit(EXIT_FAILURE);
        }

        // Create Nominal chain configuration midway between all joint limits
        this->_nominal = new KDL::JntArray(this->_chain.getNrOfJoints());

        for (uint j=0; j < this->_nominal->data.size(); j++) {
          this->_nominal->operator()(j) = (ll(j)+ul(j))/2.0;
        }

        this->_ready = true;
    }

    ~BaxterTracIKServer() {
        delete this->_tracik_solver;
        delete this->_nominal;
    }

    KDL::JntArray JointState2JntArray(const sensor_msgs::JointState &js) {
        KDL::JntArray array(this->_chain.getNrOfJoints());
        for(uint joint=0; joint<js.position.size(); ++joint) {
            array(joint) = js.position[joint];
        }
        return array;
    }

    bool perform_ik(trac_ik_baxter::GetConstrainedPositionIK::Request &request,
                    trac_ik_baxter::GetConstrainedPositionIK::Response &response) {

          if(!this->_ready)
              return false;

          double initial_tolerance = 1e-5;
          if(request.num_steps == 0)
              request.num_steps = 1;
          if(request.end_tolerance < initial_tolerance) {
              request.num_steps = 1;
              ROS_WARN("Invalid IK end tolerance, using the default");
          }
          double step = (request.end_tolerance - initial_tolerance)/request.num_steps;

          int rc;
          KDL::JntArray result;
          sensor_msgs::JointState joint_state;
          for(uint segment=0; segment<this->_chain.getNrOfSegments(); ++segment) {
              KDL::Joint joint = this->_chain.getSegment(segment).getJoint();
              if(joint.getType()!=KDL::Joint::None)
                  joint_state.name.push_back(joint.getName());
          }
          bool seeds_provided = request.seed_angles.size() == request.pose_stamp.size();

          for(uint point=0; point<request.pose_stamp.size(); ++point) {
              KDL::Frame end_effector_pose(KDL::Rotation::Quaternion(request.pose_stamp[point].pose.orientation.x,
                                                                     request.pose_stamp[point].pose.orientation.y,
                                                                     request.pose_stamp[point].pose.orientation.z,
                                                                     request.pose_stamp[point].pose.orientation.w),
                                           KDL::Vector(request.pose_stamp[point].pose.position.x,
                                                       request.pose_stamp[point].pose.position.y,
                                                       request.pose_stamp[point].pose.position.z));

              KDL::JntArray seed(this->_chain.getNrOfJoints());
              if(seeds_provided)
                  seed = JointState2JntArray(request.seed_angles[point]);

              for(uint num_attempts=0; num_attempts<request.num_steps; ++num_attempts) {
                  std::stringstream ss;
                  double tolerance = initial_tolerance + num_attempts*step;
                  ss << "Attempt num " << num_attempts +1 << " with tolerence " << tolerance << std::endl;
                  ROS_INFO("%s", ss.str().c_str());
                  //this->_tracik_solver->setEpsilon(tolerance);
                  rc = this->_tracik_solver->CartToJnt(seeds_provided? seed: *(this->_nominal),
                                                       end_effector_pose, result);
                  if(rc>=0) break;
              }

              for(uint joint=0; joint<this->_chain.getNrOfJoints(); ++joint) {
                  joint_state.position.push_back(result(joint));
              }

              response.joints.push_back(joint_state);
              response.isValid.push_back(rc>=0);
          }
          return true;
        }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "trac_ik_baxter");
  ros::NodeHandle nh;

  std::string urdf_param;
  double timeout;
  
  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  BaxterTracIKServer left("left", timeout, urdf_param);
  ros::ServiceServer left_service = nh.advertiseService("trac_ik_left", &BaxterTracIKServer::perform_ik, &left);

  BaxterTracIKServer right("right", timeout, urdf_param);
  ros::ServiceServer right_service = nh.advertiseService("trac_ik_right", &BaxterTracIKServer::perform_ik, &right);

  ros::spin();

  return 0;
}
