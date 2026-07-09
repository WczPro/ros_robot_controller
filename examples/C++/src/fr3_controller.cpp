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

#include "fr3_controller.hpp"
#include <sys/select.h>

#ifndef ROBOTS_DIRECTORY
#  error "ROBOTS_DIRECTORY is not defined. Define in CMake: -DROBOTS_DIRECTORY=\"${CMAKE_SOURCE_DIR}/../robots\""
#endif

// 按键	运动
// ↑	末端 +Y 平移 (5cm)
// ↓	末端 -Y 平移 (5cm)
// ←	末端 -X 平移 (5cm)
// →	末端 +X 平移 (5cm)
// W	绕末端 +X 旋转 10°
// S	绕末端 -X 旋转 10°
// A	绕末端 -Z 旋转 10°
// D	绕末端 +Z 旋转 10°
// 1-4	原有模式不变
// Q	退出

FR3Controller::FR3Controller(const double dt)
: dt_(dt)
{
    // Paths to URDF/SRDF (model files)
    const std::string urdf = std::string(ROBOTS_DIRECTORY) + "/fr3/" + "fr3.urdf";
    const std::string srdf = std::string(ROBOTS_DIRECTORY) + "/fr3/" + "fr3.srdf";

    // Instantiate dyros robot model/controller
    robot_data_ = std::make_shared<drc::Manipulator::RobotData>(dt_, urdf, srdf);
    robot_controller_ = std::make_shared<drc::Manipulator::RobotController>(robot_data_);

    // Degree of freedom 
    dof_ = robot_data_->getDof();

    // --- Joint-space states (initialize to zero/snapshot defaults) ---
    q_.setZero(dof_);      
    qdot_.setZero(dof_);
    q_desired_.setZero(dof_);  
    qdot_desired_.setZero(dof_);
    q_init_.setZero(dof_);
    qdot_init_.setZero(dof_);
    tau_desired_.setZero(dof_);

    // --- Task-space states (EE pose/twist and snapshots) ---
    link_ee_task_[ee_link_name_] = drc::TaskSpaceData::Zero();

    // joint_kp_ / joint_kv_
    // ├─ 大关节（肩/肘）→ Kp 大（惯量大、需要更大的恢复力）
    // ├─ 小关节（腕部）→ Kp 小（惯量小，太大会抖）
    // └─ Kp/Kv 比 ≈ 20:1 → 近似临界阻尼（Kv = 2*sqrt(Kp*I)）

    // task_ik_kp_（逆运动学的位置反馈）
    // └─ 10~50：太小响应慢，太大产生抖动

    // task_id_kp_ / task_id_kv_（逆动力学的 PD 反馈）
    // └─ 比 IK 大得多（600 / 20）：因为 ID 考虑了动力学补偿，可以更激进

    // QP 权重（qpik_tracking_ / qpik_vel_damping_ 等）
    // ├─ tracking 大 → 更看重末端跟踪
    // ├─ vel_damping 大 → 更压低关节速度（更保守，运动更慢）
    // └─ acc_damping 大 → 更压低关节加速度（更平滑，但响应更慢）

    // --- Gain
    joint_kp_.setZero(dof_);
    joint_kv_.setZero(dof_);
    qpik_vel_damping_.setZero(dof_);
    qpik_acc_damping_.setZero(dof_);
    qpid_vel_damping_.setZero(dof_);
    qpid_acc_damping_.setZero(dof_);
    // ── 关节空间 PD 反馈（按关节逐元素设置，7 个值）
    joint_kp_        << 600.0, 600.0, 600.0, 600.0, 250.0, 150.0,  50.0;
    joint_kv_        <<  30.0,  30.0,  30.0,  30.0,  10.0,  10.0,   5.0;
    // ── 任务空间 PD 反馈（6 维：3 平移 + 3 旋转）
    task_ik_kp_      <<  10.0,  10.0,  10.0,  30.0,  30.0,  30.0;
    task_id_kp_      << 600.0, 600.0, 600.0,1000.0,1000.0,1000.0;
    task_id_kv_      <<  20.0,  20.0,  20.0,  30.0,  30.0,  30.0;
    // ── QP 控制的目标权重
    qpik_tracking_   <<  10.0,  10.0,  10.0,  40.0,  40.0,  40.0;
    // ── QP 正则化（阻尼）权重（7 维，按关节）
    qpik_vel_damping_ << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    qpik_acc_damping_ << 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001;
    qpid_tracking_   <<  10.0,  10.0,  10.0,   1.0,   1.0,   1.0;
    qpid_vel_damping_ <<  0.1,   0.1,   0.1,   0.1,   0.1,   0.1,   0.1;
    qpid_acc_damping_ <<  5.0,   5.0,   5.0,   5.0,   5.0,   5.0,   5.0;
    // ── 把增益推送给底层控制器
    robot_controller_->setJointGain(joint_kp_, joint_kv_);
    robot_controller_->setIKGain(task_ik_kp_);
    robot_controller_->setIDGain(task_id_kp_, task_id_kv_);
    robot_controller_->setQPIKGain(qpik_tracking_, qpik_vel_damping_, qpik_acc_damping_);
    robot_controller_->setQPIDGain(qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);


    // Print FR3 URDF info
    std::cout << "info: \n" << robot_data_->getVerbose() << std::endl; 
    // ↑ 打印: 关节名/link 名/自由度/碰撞对等等（验证 URDF 是否被正确解析）

    // Global keyboard listener (non-blocking)
    startKeyListener_();
}

FR3Controller::~FR3Controller() 
{
    stopKeyListener_();
}

void FR3Controller::updateModel(const double current_time,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qpos_dict,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qvel_dict)
{
    // Time update (shared convention)
    sim_time_ = current_time;

    // Read joint states (joint naming must match the simulator)
    for (size_t i = 0; i < dof_; ++i) 
    {
    const std::string key = "fr3_joint" + std::to_string(i+1);
    q_(i)  = qpos_dict.at(key)[0];
    qdot_(i) = qvel_dict.at(key)[0];
    }

    // Push to dyros robot model and cache EE pose/twist
    // 推送给底层 dyros_robot_controller
    robot_data_->updateState(q_, qdot_);

    // 更新末端任务空间的「当前状态」
    link_ee_task_[ee_link_name_].x    = robot_data_->getPose(ee_link_name_);
    link_ee_task_[ee_link_name_].xdot = robot_data_->getVelocity(ee_link_name_);
    link_ee_task_[ee_link_name_].current_time = sim_time_;
}

std::unordered_map<std::string, double> FR3Controller::compute() 
{
    //关键设计：「快照」机制（Snapshot）
    // One-time init per mode entry (snapshot current measured states)
    if (is_mode_changed_) 
    {
        is_mode_changed_ = false;
        control_start_time_ = sim_time_;

        // Snapshot current measured states as new references
        q_init_ = q_;
        qdot_init_ = qdot_;
        link_ee_task_[ee_link_name_].setInit();

        // Reset desired trajectories to snapshots
        q_desired_ = q_init_;
        qdot_desired_.setZero(dof_);
        link_ee_task_[ee_link_name_].setDesired();
        link_ee_task_[ee_link_name_].xdot_desired.setZero();
    }

    // --- Mode: Home (joint-space cubic to a predefined posture) ---
    if (control_mode_ == "Home") 
    {
        Eigen::Vector7d q_home;
        // ── 定义目标：FR3 的"零位"姿态（某些关节偏置以避免奇异）
        q_home << 0.0, 0.0, 0.0, -M_PI/2., 0.0, M_PI/2., M_PI / 4.;
        // ── 三次插值计算当前时刻的 q_desired
        q_desired_ = robot_controller_->moveJointPositionCubic(q_home,
                                                               Eigen::VectorXd::Zero(dof_),
                                                               q_init_,
                                                               qdot_init_,
                                                               sim_time_,
                                                               control_start_time_,
                                                               3.0);
        // ── 三次插值计算当前时刻的 qdot_desired
        qdot_desired_ = robot_controller_->moveJointVelocityCubic(q_home,
                                                                  Eigen::VectorXd::Zero(dof_),
                                                                  q_init_,
                                                                  qdot_init_,
                                                                  sim_time_,
                                                                  control_start_time_,
                                                                  3.0);
        // ── 用 PD 反馈把 (q_desired, qdot_desired) 转换为力矩
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);

        // Alternative: synthesize torque over time with a single API
        // tau_desired = robot_controller->moveJointTorqueCubic(...)
    }
    // --- Mode: QPIK (task-space, QP-based IK with cubic profiling) ---
    else if (control_mode_ == "QPIK")
    {
        // ── 定义目标末端位姿: 当前位置 + (+10cm Y, +10cm Z)
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += Eigen::Vector3d(0.0, 0.1, 0.1); // +10 cm in Y and Z

        robot_controller_->QPIKCubic(link_ee_task_,
                                     3.0,
                                     qdot_desired_);

        // Simple Euler integrate desired joint positions from qdot_desired
        q_desired_   = q_ + qdot_desired_ * dt_;

        // Map (q, qdot) -> torque (PD + gravity)
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame translation: 沿着末端坐标系 +Y 方向平移 5cm（↑ 上方向键） ──
    else if (control_mode_ == "QPIK_Up")
    {
        const Eigen::Matrix3d R_ee = link_ee_task_[ee_link_name_].x_init.rotation();
        const Eigen::Vector3d offset_world = R_ee * Eigen::Vector3d(0.0, 0.05, 0.0); // +5cm in EE Y
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += offset_world;
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_   = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame translation: 沿着末端坐标系 -Y 方向平移 5cm（↓ 下方向键） ──
    else if (control_mode_ == "QPIK_Down")
    {
        const Eigen::Matrix3d R_ee = link_ee_task_[ee_link_name_].x_init.rotation();
        const Eigen::Vector3d offset_world = R_ee * Eigen::Vector3d(0.0, -0.05, 0.0); // -5cm in EE Y
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += offset_world;
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_   = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame translation: 沿着末端坐标系 -X 方向平移 5cm（← 左方向键） ──
    else if (control_mode_ == "QPIK_Left")
    {
        const Eigen::Matrix3d R_ee = link_ee_task_[ee_link_name_].x_init.rotation();
        const Eigen::Vector3d offset_world = R_ee * Eigen::Vector3d(-0.05, 0.0, 0.0); // -5cm in EE X
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += offset_world;
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_   = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame translation: 沿着末端坐标系 +X 方向平移 5cm（→ 右方向键） ──
    else if (control_mode_ == "QPIK_Right")
    {
        const Eigen::Matrix3d R_ee = link_ee_task_[ee_link_name_].x_init.rotation();
        const Eigen::Vector3d offset_world = R_ee * Eigen::Vector3d(0.05, 0.0, 0.0); // +5cm in EE X
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += offset_world;
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_   = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame rotation: 沿着末端坐标系 +X 轴转动 10°（W 按键） ──
    else if (control_mode_ == "QPIK_Pitch_Forward")
    {
        const double angle = 10.0 * M_PI / 180.0;
        const Eigen::AngleAxisd delta_rot(angle, Eigen::Vector3d::UnitX());
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() =
            link_ee_task_[ee_link_name_].x_init.linear() * delta_rot.toRotationMatrix();
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_   = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame rotation: 沿着末端坐标系 -X 轴转动 10°（S 按键） ──
    else if (control_mode_ == "QPIK_Pitch_Anti")
    {
        const double angle = -10.0 * M_PI / 180.0;
        const Eigen::AngleAxisd delta_rot(angle, Eigen::Vector3d::UnitX());
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() =
            link_ee_task_[ee_link_name_].x_init.linear() * delta_rot.toRotationMatrix();
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_   = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame rotation: 沿着末端坐标系 -Z 轴转动 10°（A 按键） ──
    else if (control_mode_ == "QPIK_Roller_Anti")
    {
        const double angle = -10.0 * M_PI / 180.0;
        const Eigen::AngleAxisd delta_rot(angle, Eigen::Vector3d::UnitZ());
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() =
            link_ee_task_[ee_link_name_].x_init.linear() * delta_rot.toRotationMatrix();
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_   = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame rotation: 沿着末端坐标系 +Z 轴转动 10°（D 按键） ──
    else if (control_mode_ == "QPIK_Roller_Forward")
    {
        const double angle = 10.0 * M_PI / 180.0;
        const Eigen::AngleAxisd delta_rot(angle, Eigen::Vector3d::UnitZ());
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() =
            link_ee_task_[ee_link_name_].x_init.linear() * delta_rot.toRotationMatrix();
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_   = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // --- Mode: Gravity Compensation (no tracking) ---
    else if (control_mode_ == "Gravity Compensation") 
    {
        tau_desired_ = robot_data_->getGravity();
    }
    // --- Mode: Gravity Compensation W QPID (no tracking) ---
    else if (control_mode_ == "Gravity Compensation W QPID") 
    {
        link_ee_task_[ee_link_name_].xddot_desired.setZero();
        robot_controller_->QPID(link_ee_task_, tau_desired_);
    }

    // Format output for simulator actuators
    std::unordered_map<std::string, double> ctrl_dict;
    ctrl_dict.reserve(dof_);
    for (size_t i = 0; i < dof_; ++i) 
    {
        ctrl_dict.emplace("fr3_joint" + std::to_string(i+1), tau_desired_(i));
        //          ↑ 关节名必须与 MuJoCo 场景文件中 actuator 的 name 一致
    }
    return ctrl_dict;
}

void FR3Controller::setMode(const std::string& control_mode) 
{
    // Switch control mode and trigger per-mode re-initialization
    is_mode_changed_ = true;
    control_mode_ = control_mode;
    std::cout << "Control Mode Changed: " << control_mode_ << std::endl;
}

void FR3Controller::startKeyListener_() 
{
    tty_ok_ = ::isatty(STDIN_FILENO);
    if (!tty_ok_) 
    {
        std::cout << "[FR3Controller] stdin is not a TTY; keyboard control disabled.\n";
        return;
    }
    // 把终端设为 "原始模式"（关键！）
    setRawMode_();
    // ↑ 正常终端是 "规范模式": 输入被缓冲直到回车
    //   原始模式: 每一个按键立即被 read() 返回，无需回车
    //             且不回显（按键不显示在屏幕上）

    // 启动后台线程
    stop_key_ = false;
    key_thread_ = std::thread(&FR3Controller::keyLoop_, this);
    // ↑ 线程入口: keyLoop_() 成员函数
    //   注意: std::thread 的第一个参数是成员函数指针，第二个是 this 指针


    std::cout << "[FR3Controller] Keyboard:\n"
              << "  [1] Home  [2] QPIK  [3] Gravity Comp  [4] Gravity Comp W QPID\n"
              << "  [ArrowUp] EE +Y    [ArrowDown] EE -Y    [ArrowLeft] EE -X    [ArrowRight] EE +X\n"
              << "  [W] EE +X rot      [S] EE -X rot        [A] EE -Z rot        [D] EE +Z rot\n"
              << "  [Q] Quit\n";
}

void FR3Controller::stopKeyListener_() 
{
    if (!tty_ok_) return;
    stop_key_ = true;
    if (key_thread_.joinable()) key_thread_.join();
    restoreTerm_();
}

void FR3Controller::setRawMode_() 
{
    if (!tty_ok_) return;
    struct termios raw;
    if (tcgetattr(STDIN_FILENO, &orig_term_) == -1) 
    {
        perror("tcgetattr");
        tty_ok_ = false;
        return;
    }
    raw = orig_term_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 1;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) 
    {
        perror("tcsetattr");
        tty_ok_ = false;
    }
}

void FR3Controller::restoreTerm_() 
{
    if (!tty_ok_) return;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_) == -1) 
    {
        perror("tcsetattr restore");
    }
}

void FR3Controller::keyLoop_()
{
    while (!stop_key_)
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 20 * 1000; // 20 ms poll interval

        int ret = ::select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds))
        {
            char buf[8];
            ssize_t nread = ::read(STDIN_FILENO, buf, sizeof(buf));
            for (ssize_t i = 0; i < nread; ++i)
            {
                char c = buf[i];
                // ── Numeric modes ──
                if (c == '1')              setMode("Home");
                else if (c == '2')         setMode("QPIK");
                else if (c == '3')         setMode("Gravity Compensation");
                else if (c == '4')         setMode("Gravity Compensation W QPID");
                // ── EE-frame translations ──
                else if (c == '\x1b' && i + 2 < nread && buf[i+1] == '[') // Arrow keys
                {
                    char code = buf[i+2];
                    if      (code == 'A')      setMode("QPIK_Up");
                    else if (code == 'B')      setMode("QPIK_Down");
                    else if (code == 'C')      setMode("QPIK_Right");
                    else if (code == 'D')      setMode("QPIK_Left");
                    i += 2; // skip [ and code byte
                }
                // ── EE-frame rotations (WASD) ──
                else if (c == 'w' || c == 'W') setMode("QPIK_Pitch_Forward");
                else if (c == 's' || c == 'S') setMode("QPIK_Pitch_Anti");
                else if (c == 'a' || c == 'A') setMode("QPIK_Roller_Anti");
                else if (c == 'd' || c == 'D') setMode("QPIK_Roller_Forward");
                // ── Quit ──
                else if (c == 'q' || c == 'Q') stop_key_ = true;
            }
        }
    }
}
