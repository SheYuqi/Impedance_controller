// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef IMPEDANCE_CONTROLLER_HPP
#define IMPEDANCE_CONTROLLER_HPP


#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/Speaker.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Display.hpp>

#include <Eigen/Dense>
#include <pinocchio/multibody/sample-models.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/motion.hpp>
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/utils/timer.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>

using namespace webots;
using namespace std;
using namespace pinocchio;

class KukaLBRIIWAPlayer : public Supervisor
{
public:
  KukaLBRIIWAPlayer(Model &_model);
  virtual ~KukaLBRIIWAPlayer();
  void myStep();
  void wait(int ms);
  void init();
  void run();
  void computeFeedforwardPID();
  void computeImpedanceControl(int eeFrameId);
  void computeJacobianDot(Eigen::MatrixXd &J_dot,int eeFrameId);
  Eigen::Matrix3d axisAngleToRotationMatrix(const double *rotation);
  void loadConfig(const std::string &yamlFile);
  void inverseKinematics(pinocchio::Model &model, 
                       pinocchio::Data &data, 
                       const Eigen::VectorXd &initialConfiguration, 
                       const pinocchio::SE3 &targetPose, 
                       int eeFrameId, 
                       Eigen::VectorXd &resultConfiguration, 
                       double tolerance = 1e-4, 
                       int maxIterations = 100);
  
private:
  int mTimeStep;
  Model model;
  Motor *mMotors[7];
  PositionSensor *mPositionSensors[7];
  Data data;              // Pinocchio 数据
  Eigen::VectorXd q;      // 机器人关节配置（角度）
  Eigen::VectorXd q_dot;  // 关节速度
  Eigen::VectorXd q_ddot; // 关节加速度
  Eigen::VectorXd q_dot_last;
  Eigen::VectorXd q_last;
  Eigen::VectorXd tau_ff; // 前馈力矩
  Eigen::VectorXd targetAcceleration;
  Eigen::VectorXd targetAngles;

  // 阻抗控制相关变量
  Eigen::MatrixXd M;  // 惯性矩阵 (7x7)
  Eigen::MatrixXd D;  // 阻尼矩阵 (7x7)
  Eigen::MatrixXd K;  // 刚度矩阵 (7x7)
  Eigen::VectorXd f_ext; // 外部作用力 (7x1)

  // 期望末端状态
  Eigen::VectorXd x_des;  // 期望位置 (3x1)
  Eigen::VectorXd v_des;  // 期望速度 (3x1)
  Eigen::VectorXd a_des;  // 期望加速度 (3x1)

  // 当前末端状态
  Eigen::VectorXd x;  // 当前末端位置 (3x1)
  Eigen::VectorXd x_last;  // 上一末端位置 (3x1)
  Eigen::VectorXd v;  // 当前末端速度 (3x1)
  Eigen::VectorXd a;  // 当前末端加速度 (3x1) 

  // Supervisor supervisor;
  // 初始化关节配置、速度和加速度

  double Kp[7]={200, 200, 200,200, 200, 200, 200};
  double Ki[7]={0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  double Kv[7]={20, 20, 20, 20, 20, 20, 20};
  double Kb = 20;
  double Kk = 20;
  double Km = 20;
  double integral[7], previous_error[7];
  //double targetAngles[7]={1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  double targetVelocity[7]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  //double targetAcceleration[7]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

#endif  // FEEDFORWARD_CONTROLLER_HPP
