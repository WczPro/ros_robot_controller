// Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
//
// Developed by Yoon Junheon at the Dynamic Robotic Systems Laboratory (DYROS),
// Seoul National University, under a research agreement with ETRI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/manipulator/robot_controller.h"

class UR8ControllerQPID
{
    public:
        UR8ControllerQPID(const double dt);
        ~UR8ControllerQPID();
        void updateModel(const double current_time,
                         const std::unordered_map<std::string, Eigen::VectorXd>& qpos_dict,
                         const std::unordered_map<std::string, Eigen::VectorXd>& qvel_dict);
        std::unordered_map<std::string, double> compute();
        void setMode(const std::string& control_mode);

    private:
        const double dt_;
        int dof_{0};

        // Joint-space states
        Eigen::VectorXd q_;
        Eigen::VectorXd qdot_;
        Eigen::VectorXd q_desired_;
        Eigen::VectorXd qdot_desired_;
        Eigen::VectorXd q_init_;
        Eigen::VectorXd qdot_init_;
        Eigen::VectorXd tau_desired_;

        // Joint name mapping (must match Pinocchio joint order)
        std::vector<std::string> joint_names_;

        // Task-space states
        std::map<std::string, drc::TaskSpaceData> link_ee_task_;
        std::string ee_link_name_{"link_tool"};

        // Mode bookkeeping
        std::string control_mode_{"Pose Knife"};
        bool   is_mode_changed_{true};
        double sim_time_{0.0};
        double control_start_time_{0.0};

        // Gains
        Eigen::VectorXd joint_kp_;
        Eigen::VectorXd joint_kv_;
        Vector6d        task_ik_kp_;
        Vector6d        task_id_kp_;
        Vector6d        task_id_kv_;
        Vector6d        qpik_tracking_;
        Eigen::VectorXd qpik_vel_damping_;
        Eigen::VectorXd qpik_acc_damping_;
        Vector6d        qpid_tracking_;
        Eigen::VectorXd qpid_vel_damping_;
        Eigen::VectorXd qpid_acc_damping_;

        // Dyros model/controller handles
        std::shared_ptr<drc::Manipulator::RobotData>       robot_data_;
        std::shared_ptr<drc::Manipulator::RobotController> robot_controller_;

        // Keyboard interface
        void startKeyListener_();
        void stopKeyListener_();
        void keyLoop_();
        void setRawMode_();
        void restoreTerm_();

        std::atomic<bool> stop_key_{false};
        std::thread key_thread_;
        bool tty_ok_{false};
        struct termios orig_term_{};
};
