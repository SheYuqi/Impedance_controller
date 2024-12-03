#include "impedance_controller.hpp"

#include <iostream>

using namespace webots;
using namespace std;
using namespace pinocchio;

static const char *motorNames[7] = {
    "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"};

KukaLBRIIWAPlayer::KukaLBRIIWAPlayer(Model &_model) : Supervisor()
{
  // 获取时间步长

  mTimeStep = getBasicTimeStep();
  // 初始化Pinocchio模型和数据
  model = _model;
  cout << "--- feedforward controller of KUKA LBR iiwa ---" << endl;
  data = pinocchio::Data(model);

  // 获取电机和位置传感器
  for (int i = 0; i < 7; i++)
  {
    mMotors[i] = getMotor(motorNames[i]);
    mPositionSensors[i] = getPositionSensor(motorNames[i] + string("_sensor"));
    mPositionSensors[i]->enable(mTimeStep);
    usleep(10);

    // 初始化 PID 参数

    // 初始化误差和积分值
    integral[i] = 0.0;
    previous_error[i] = 0.0;
  }

  step(mTimeStep);
  // 初始化关节配置、速度和加速度
  q = Eigen::VectorXd::Zero(7);
  q_dot = Eigen::VectorXd::Zero(7);
  q_dot_last = Eigen::VectorXd::Zero(7);
  q_ddot = Eigen::VectorXd::Zero(7);
  q_last = Eigen::VectorXd::Zero(7);
  tau_ff = Eigen::VectorXd::Zero(7);
  targetAcceleration = Eigen::VectorXd::Zero(7);
  targetAngles = Eigen::VectorXd::Zero(7);

  x_last = Eigen::VectorXd::Zero(6);
  x_des = Eigen::VectorXd::Zero(6);
  v_des = Eigen::VectorXd::Zero(6);
  a_des = Eigen::VectorXd::Zero(6);
  x = Eigen::VectorXd::Zero(6);
  v = Eigen::VectorXd::Zero(6);
  a = Eigen::VectorXd::Zero(6);
  f_ext = Eigen::VectorXd::Zero(6);
  
 
  for (int i = 0; i < 7; i++)
  {
    q_last(i) = mPositionSensors[i]->getValue();
  }
}

KukaLBRIIWAPlayer::~KukaLBRIIWAPlayer()
{
  // 这里可以添加清理代码，如果有资源需要释放
}

void KukaLBRIIWAPlayer::myStep()
{
  if (step(mTimeStep) == -1)
    exit(EXIT_SUCCESS);
}

void KukaLBRIIWAPlayer::wait(int ms)
{
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void KukaLBRIIWAPlayer::loadConfig(const std::string &yamlFile)
{
  YAML::Node config = YAML::LoadFile(yamlFile);

  // targetAngles.translation() = config["targetAngles"];
  Kb  = config["Kb"].as<double>();
  Kk  = config["Kk"].as<double>();
  Km  = config["Km"].as<double>();
  std::cout << "Kb: " << Kb << std::endl;
  std::cout << "Kk: " << Kk << std::endl;
  std::cout << "Km: " << Km << std::endl;
  for (int i = 0; i < 7; i++)
  {
    //targetVelocity[i] = config["targetVelocity"][0].as<double>();
    //Kp[i] = config["Kp"][0].as<double>();
    //Kv[i] = config["Kv"][0].as<double>();
  }
}

Eigen::Matrix3d KukaLBRIIWAPlayer::axisAngleToRotationMatrix(const double *rotation) {
  Eigen::Vector3d axis(rotation[0], rotation[1], rotation[2]);
  double angle = rotation[3];
  Eigen::AngleAxisd angleAxis(angle, axis);  //这是什么通过四元数求出旋转矩阵的方法？
  return angleAxis.toRotationMatrix();
}


void KukaLBRIIWAPlayer::computeFeedforwardPID()
{

  cout << "--- Controll start ---" << endl;
  // 读取关节角度和速度
  for (int i = 0; i < 7; i++)
  {
    q(i) = mPositionSensors[i]->getValue();
  }
  pinocchio::forwardKinematics(model, data, q); // 每次获取得到角度后必须更新
  q_dot = 1000 * (q - q_last) / mTimeStep;
  // q_ddot = 1000*(q_dot - q_dot_last)/mTimeStep;

  // PID 控制   //计算q_ddot
  for (int i = 0; i < 7; i++)
  {
    double error = targetAngles(i) - q(i); // 计算目标位置与当前的位置误差
    double derivative = targetVelocity[i] - q_dot(i);
    cout << "error: " << error << endl
         << "derivative: " << derivative << endl;
    double tau_pid = Kp[i] * error + Kv[i] * derivative;
    cout << "tau_pid: " << tau_pid << endl;
    q_ddot(i) = targetAcceleration(i) + tau_pid;
    // mMotors[i]->setTorque(tau_pid);
  }
  tau_ff = rnea(model, data, q, q_dot, q_ddot);
  // tau_ff = Eigen::VectorXd::Zero(7);
  cout << "tau_ff: " << tau_ff.transpose() << endl;
  cout << "q: " << q.transpose() << endl
       << "q_dot: " << q_dot.transpose() << endl
       << "q_ddot: " << q_ddot.transpose() << endl;

  for (int i = 0; i < 7; i++)
  {
    mMotors[i]->setTorque(tau_ff(i));
  }

  q_last = q;
  q_dot_last = q_dot;
}

void KukaLBRIIWAPlayer::computeJacobianDot(Eigen::MatrixXd &J_dot,int eeFrameId) {
  // 数值差分计算雅可比矩阵的导数
  double delta_q = 0;
  delta_q = 1e-3;
  Eigen::VectorXd q_plus = q;
  Eigen::VectorXd q_minus = q;

  for (int i = 0; i < model.nv; ++i) {
      // 微小变化
      q_plus(i) += delta_q; // delta_q 为一个非常小的增量
      q_minus(i) -= delta_q;
      // 计算在 q+ 和 q- 时的雅可比矩阵
  }

  Eigen::MatrixXd J_plus(6, model.nv);
  Eigen::MatrixXd J_minus(6, model.nv);
  pinocchio::computeFrameJacobian(model, data, q_plus, eeFrameId, J_plus);
  pinocchio::computeFrameJacobian(model, data, q_minus, eeFrameId, J_minus);
      // 数值计算导数
  for (int i = 0; i < model.nv; i++) {
  J_dot.col(i) = (J_plus.col(i) - J_minus.col(i)) / (2 * delta_q);
  }
  cout << "J_dot: " << J_dot << endl;
}

void KukaLBRIIWAPlayer::computeImpedanceControl(int eeFrameId) {
  // 更新当前末端位置和速度
  cout << "--- ImpedanceControl start ---" << endl;
  // 读取关节角度和速度
  for (int i = 0; i < 7; i++)
  {
    q(i) = mPositionSensors[i]->getValue();
  }
  q_dot = 1000 * (q - q_last) / mTimeStep;
  q_ddot = 1000 * (q_dot - q_dot_last) / mTimeStep;
  pinocchio::forwardKinematics(model, data, q); // 每次获取得到角度后必须更新
  pinocchio::updateFramePlacements(model, data);
  // 获取目标末端的速度
  std::cout << "data.oMf[eeFrameId]:" << data.oMf[eeFrameId] << std::endl;

  q_last = q;
  q_dot_last = q_dot;
  //x << data.oMf[eeFrameId].translation(),pinocchio::log3(data.oMf[eeFrameId].rotation());  
  x << data.oMf[eeFrameId].translation(),0,0,0;
  x_last = x;
  Eigen::MatrixXd J(6, model.nv); // 关节雅可比矩阵
  J.setZero();
  pinocchio::computeFrameJacobian(model, data, q, eeFrameId, J);
  Eigen::MatrixXd J_dot(6, model.nv); // 雅可比矩阵导数
  J_dot.setZero();
  v = J * q_dot;
  a = J * q_ddot + J_dot * q_dot;
   
  //a = J * q_ddot; // 关节加速度
  // Eigen::MatrixXd M_q = pinocchio::crba(model, data, q);
  // Eigen::MatrixXd M_x = (J * M_q.inverse() * J.transpose()).inverse();

  // 计算阻抗控制力
  Eigen::VectorXd x_error = x_des - x; // 位置误差
  Eigen::VectorXd v_error = v_des - v; // 速度误差
  Eigen::VectorXd f_impedance = K * x_error + D * v_error + M * (a_des-a); // 阻抗力

  // 转换为关节空间的力矩
  
  Eigen::VectorXd tau = J.transpose() * f_impedance; // 转换为关节力矩

  // 更新控制输入
  for (int i = 0; i < 7; i++) {
    mMotors[i]->setTorque(tau(i));
  }
  cout << "tau: " << tau.transpose() << endl
       << "x_error: " << x_error.transpose() << endl
       << "v_error: " << v_error.transpose() << endl
       << "x: " << x.transpose() << endl
       << "v: " << v.transpose() << endl
       << "J: " << J << endl
       << "a: " << a.transpose() << endl
       << "x_des: " << x_des.transpose() << endl
       << "v_des: " << v_des.transpose() << endl
       << "f_impedance: " << f_impedance.transpose() << endl;
}

void KukaLBRIIWAPlayer::inverseKinematics(pinocchio::Model &model, pinocchio::Data &data, const Eigen::VectorXd &initialConfiguration,
                                          const pinocchio::SE3 &targetPose, int eeFrameId, Eigen::VectorXd &resultConfiguration, double tolerance, int maxIterations)
{
  const double DT = 1e-1;
  const double damp = 1e-6;
  const double eps = tolerance; // Tolerance for convergence
  // Initialize configuration with the initial guess
  Eigen::VectorXd _q = initialConfiguration;
  Eigen::VectorXd v(model.nv);

  std::cout << "eeFrameId:" << eeFrameId << std::endl;

  for (int iter = 0; iter < maxIterations; ++iter)
  {
    // Forward kinematics to compute the current end-effector pose
    pinocchio::forwardKinematics(model, data, _q);
    pinocchio::updateFramePlacements(model, data); // Update frame placements

    // Get the current pose of the end-effector
    pinocchio::SE3 currentPose = data.oMf[eeFrameId];
    // std::cout << "currentPose: " << currentPose << std::endl;

    // Compute the pose error (logarithm map of SE3)
    pinocchio::SE3 errorSE3 = currentPose.actInv(targetPose);
    Eigen::VectorXd error = pinocchio::log6(errorSE3).toVector();

    // Check convergence
    if (error.norm() < eps)
    {
      resultConfiguration = _q;
      std::cout << "Inverse Kinematics converged successfully!" << std::endl;
      std::cout << "Resulting configuration: " << targetAngles.transpose() << std::endl;
      return; // Converged successfully
    }

    // Compute the Jacobian of the end-effector
    Eigen::MatrixXd J(6, model.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, _q, eeFrameId, J);
    //std::cout << "error: " << error.norm() << std::endl;
    // Solve for the configuration update
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(errorSE3.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(error);
    _q = pinocchio::integrate(model, _q, v * DT);
  }
  std::cout << "_q: " << _q.transpose() << std::endl;
  std::cout << "v: " << v.transpose() << std::endl;

  // If the loop exits, it means IK did not converge
  std::cout << "Inverse Kinematics did not converge." << std::endl;

  return;
}

void KukaLBRIIWAPlayer::init()
{
  for (int i = 0; i < 7; i++)
  {
    mMotors[i]->setPosition(0);
    mMotors[i]->setVelocity(0.0); // 设置电机速度
  }
  // 初始化动作
}
void KukaLBRIIWAPlayer::run()
{
  loadConfig("/home/syq/my_project/controllers/impedance_controller/src/config.yaml");//src/config.yaml
  D = Eigen::MatrixXd::Identity(6, 6) * Kb; // 阻尼矩阵 (临界阻尼)
  K = Eigen::MatrixXd::Identity(6, 6) * Kk; // 刚度矩阵 (高刚度)
  M = Eigen::MatrixXd::Identity(6, 6) * Km; // 惯性矩阵 (单位质量)
  Eigen::VectorXd initialConfiguration = Eigen::VectorXd::Zero(model.nq);
  initialConfiguration << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3; // 如果初始值为0，则会导致IK求解出的数值不在合理范围内
  x << 0,0,1,0,0,0;
                                                             //  3. 目标末端执行器的位姿
  pinocchio::SE3 targetPose = pinocchio::SE3::Identity();
  targetPose.translation() = Eigen::Vector3d(0.5, 0.3, 1.0); // 示例位置
  targetPose.rotation() = Eigen::Matrix3d::Identity();       // 示例旋转

  // 4. 获取末端执行器的frame ID
  int eeFrameId = model.getFrameId("iiwa_link_ee"); // 根据URDF中的frame名称调整  iiwa_link_ee
  std::cout << "eeFrameId:" << eeFrameId << std::endl;

  // 5. 保存结果的配置
  targetAngles = initialConfiguration;

  // 获取小球节点
  Node *ballNode = getFromDef("BALL");

  if (!ballNode)
  {
    std::cerr << "Could not find BALL node in the scene." << std::endl;
  }
  

  // 获取小球的 translation 字段
  Field *translationField = ballNode->getField("translation");
  Field *rotationField = ballNode->getField("rotation");

  //step(mTimeStep);

  while (step(mTimeStep) != -1)
  {
    const double *position = translationField->getSFVec3f();
    const double *rotation = rotationField->getSFRotation();
    targetPose.rotation() = axisAngleToRotationMatrix(rotation);
    targetPose.translation() = Eigen::Vector3d(position[0], position[1], position[2]); // 示例位置 targetPose.translation() = Eigen::Vector3d(0.5, 0.3, 1.0); // 示例位置
    // inverseKinematics(model, data, initialConfiguration, targetPose, eeFrameId, targetAngles,
    //                   1e-3, // 收敛容差
    //                   500);
    
    v_des = Eigen::VectorXd::Zero(6);
    x_des << position[0], position[1], position[2],rotation[3]*rotation[0], rotation[3]*rotation[1], rotation[3]*rotation[2];

    computeImpedanceControl(eeFrameId);
  }
}
