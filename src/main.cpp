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

// Description:   Manage the entree point function
#include "impedance_controller.hpp"

#include <iostream>
#include <stdlib.h>

using namespace webots;

int main(int argc, char **argv) {
  // 创建 KukaLBRIIWAPlayer 控制器对象
  cout << "The feedback controller is running." << endl;

  const std::string urdf_filename = std::string("/home/syq/kuka_iiwa/urdf/iiwa7.urdf");
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);
  pinocchio ::Data data = pinocchio::Data(model);
  std::cout << "model:" << model.name << std::endl;
  KukaLBRIIWAPlayer *controller = new KukaLBRIIWAPlayer(model);
  
  // 运行控制器
  controller->run();

  // 删除控制器对象，清理内存
  delete controller;

  return EXIT_FAILURE;  // 可修改为 EXIT_SUCCESS，取决于是否需要返回成功
}
