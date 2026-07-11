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

#include "ur8_controller_qpik.hpp"
#include <sys/select.h>

#ifndef ROBOTS_DIRECTORY
#  error "ROBOTS_DIRECTORY is not defined. Define in CMake: -DROBOTS_DIRECTORY=\"${CMAKE_SOURCE_DIR}/../robots\""
#endif

UR8ControllerQPIK::UR8ControllerQPIK(const double dt)
: dt_(dt)
{
    const std::string urdf = std::string(ROBOTS_DIRECTORY) + "/ur8/" + "ur8.urdf";
    const std::string srdf = std::string(ROBOTS_DIRECTORY) + "/ur8/" + "ur8.srdf";

    robot_data_ = std::make_shared<drc::Manipulator::RobotData>(dt_, urdf, srdf);
    robot_controller_ = std::make_shared<drc::Manipulator::RobotController>(robot_data_);

    dof_ = robot_data_->getDof();

    // Build joint name list matching Pinocchio joint order
    joint_names_.clear();
    for (int i = 1; i <= 8; ++i)
        joint_names_.push_back("joint" + std::to_string(i));
    joint_names_.push_back("joint_grasp_left");
    joint_names_.push_back("joint_grasp_right");

    // Joint-space states
    q_.setZero(dof_);
    qdot_.setZero(dof_);
    q_desired_.setZero(dof_);
    qdot_desired_.setZero(dof_);
    q_init_.setZero(dof_);
    qdot_init_.setZero(dof_);
    tau_desired_.setZero(dof_);

    // Task-space states
    link_ee_task_[ee_link_name_] = drc::TaskSpaceData::Zero();

    //第一组参数-----------------------------------------------------------------------//
    // // Gains — start conservative, tune up gradually
    // joint_kp_.setZero(dof_);
    // joint_kv_.setZero(dof_);
    // joint_kp_ <<  5000.0, 8000.0, 12000.0, 3000.0, 2500.0, 2000.0, 1000.0,  500.0, 200.0, 200.0;  // j1 reduced
    // joint_kv_ <<  8000.0, 6400.0,  9600.0, 2400.0, 2000.0, 1600.0,  800.0,  400.0, 160.0, 160.0;  // j1 damped

    // task_ik_kp_      <<  8.0,   8.0,   8.0,  10.0,  10.0,  10.0;   // balanced
    // task_id_kp_      << 300.0, 300.0, 300.0, 600.0, 600.0, 600.0;
    // task_id_kv_      <<  10.0,  10.0,  10.0,  15.0,  15.0,  15.0;
    // qpik_tracking_   <<  10.0,  10.0,  10.0,  10.0,  10.0,  10.0;   // uniform

    // // QPIK regularization — per-joint (j1~j8, grasp_l, grasp_r)
    // qpik_vel_damping_.setZero(dof_);
    // qpik_vel_damping_ << 1.0, 0.5, 0.8, 0.05, 0.05, 0.03, 0.02, 0.01, 0.01, 0.01;

    // qpik_acc_damping_.setZero(dof_);
    // qpik_acc_damping_ << 0.002, 0.001, 0.002, 0.0005, 0.0005, 0.0008, 0.001, 0.002, 0.002, 0.002;

    // // QPID regularization — per-joint
    // qpid_tracking_   <<  10.0,  10.0,  10.0,  30.0,  30.0,  30.0;

    // qpid_vel_damping_.setZero(dof_);
    // qpid_vel_damping_ << 1.0, 0.8, 0.8, 0.5, 0.5, 0.3, 0.2, 0.1, 0.1, 0.1;

    // qpid_acc_damping_.setZero(dof_);
    // qpid_acc_damping_ << 2.0, 3.0, 3.0, 5.0, 5.0, 8.0, 10.0, 20.0, 20.0, 20.0;

    //第二组参数-----------------------------------------------------------------------//
    // // Gains — start conservative, tune up gradually
    // joint_kp_.setZero(dof_);
    // joint_kv_.setZero(dof_);
    // joint_kp_ <<  5000.0, 8000.0, 12000.0, 3000.0, 2500.0, 2000.0, 1000.0,  500.0, 200.0, 200.0;  // j1 reduced
    // joint_kv_ <<  8000.0, 8000.0, 12000.0, 2400.0, 2000.0, 1600.0,  800.0,  400.0, 160.0, 160.0;  // j2,j3 more damped

    // task_ik_kp_      << 60.0,  60.0,  60.0,  10.0,  10.0,  10.0;   // linear authority
    // task_id_kp_      << 300.0, 300.0, 300.0, 600.0, 600.0, 600.0;
    // task_id_kv_      <<  10.0,  10.0,  10.0,  15.0,  15.0,  15.0;
    // qpik_tracking_   <<  60.0,  60.0,  60.0,  10.0,  10.0,  10.0;   // linear authority

    // // QPIK regularization — per-joint (j1~j8, grasp_l, grasp_r)
    // qpik_vel_damping_.setZero(dof_);
    // qpik_vel_damping_ << 0.5, 0.005, 0.005, 0.005, 0.005, 0.003, 0.002, 0.001, 0.001, 0.001;   // light damp

    // qpik_acc_damping_.setZero(dof_);
    // qpik_acc_damping_ << 0.002, 0.001, 0.002, 0.0005, 0.0005, 0.0008, 0.001, 0.002, 0.002, 0.002;

    // // QPID regularization — per-joint
    // qpid_tracking_   <<  10.0,  10.0,  10.0,  30.0,  30.0,  30.0;

    // qpid_vel_damping_.setZero(dof_);
    // qpid_vel_damping_ << 1.0, 0.8, 0.8, 0.5, 0.5, 0.3, 0.2, 0.1, 0.1, 0.1;

    // qpid_acc_damping_.setZero(dof_);
    // qpid_acc_damping_ << 2.0, 3.0, 3.0, 5.0, 5.0, 8.0, 10.0, 20.0, 20.0, 20.0;

    //第三组参数-----------------------------------------------------------------------//
    // Gains — start conservative, tune up gradually
    joint_kp_.setZero(dof_);
    joint_kv_.setZero(dof_);
    joint_kp_ <<  5000.0, 8000.0, 12000.0, 3000.0, 2500.0, 2000.0, 1000.0,  500.0, 200.0, 200.0;  // j1 reduced
    joint_kv_ <<  8000.0, 8000.0, 12000.0, 2400.0, 2000.0, 1600.0,  800.0,  400.0, 160.0, 160.0;  // j2,j3 more damped

    task_ik_kp_      << 25.0,  25.0,  25.0,  10.0,  10.0,  10.0;   // linear moderate, angular low
    task_id_kp_      << 300.0, 300.0, 300.0, 600.0, 600.0, 600.0;
    task_id_kv_      <<  10.0,  10.0,  10.0,  15.0,  15.0,  15.0;
    qpik_tracking_   <<  25.0,  25.0,  25.0,  10.0,  10.0,  10.0;   // linear moderate

    // QPIK regularization — per-joint (j1~j8, grasp_l, grasp_r)
    qpik_vel_damping_.setZero(dof_);
    qpik_vel_damping_ << 0.5, 0.01, 0.01, 0.01, 0.01, 0.005, 0.003, 0.001, 0.001, 0.001;   // balanced

    qpik_acc_damping_.setZero(dof_);
    qpik_acc_damping_ << 0.002, 0.001, 0.002, 0.0005, 0.0005, 0.0008, 0.001, 0.002, 0.002, 0.002;

    // QPID regularization — per-joint
    qpid_tracking_   <<  10.0,  10.0,  10.0,  30.0,  30.0,  30.0;

    qpid_vel_damping_.setZero(dof_);
    qpid_vel_damping_ << 1.0, 0.8, 0.8, 0.5, 0.5, 0.3, 0.2, 0.1, 0.1, 0.1;

    qpid_acc_damping_.setZero(dof_);
    qpid_acc_damping_ << 2.0, 3.0, 3.0, 5.0, 5.0, 8.0, 10.0, 20.0, 20.0, 20.0;   

    robot_controller_->setJointGain(joint_kp_, joint_kv_);
    robot_controller_->setIKGain(task_ik_kp_);
    robot_controller_->setIDGain(task_id_kp_, task_id_kv_);
    robot_controller_->setQPIKGain(qpik_tracking_, qpik_vel_damping_, qpik_acc_damping_);
    robot_controller_->setQPIDGain(qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);

    std::cout << "info: \n" << robot_data_->getVerbose() << std::endl;

    startKeyListener_();
}

UR8ControllerQPIK::~UR8ControllerQPIK()
{
    stopKeyListener_();
}

void UR8ControllerQPIK::updateModel(const double current_time,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qpos_dict,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qvel_dict)
{
    sim_time_ = current_time;

    for (size_t i = 0; i < dof_; ++i)
    {
        const std::string& key = joint_names_[i];
        q_(i)    = qpos_dict.at(key)[0];
        qdot_(i) = qvel_dict.at(key)[0];
    }

    robot_data_->updateState(q_, qdot_);
    link_ee_task_[ee_link_name_].x    = robot_data_->getPose(ee_link_name_);
    link_ee_task_[ee_link_name_].xdot = robot_data_->getVelocity(ee_link_name_);
    link_ee_task_[ee_link_name_].current_time = sim_time_;
}

std::unordered_map<std::string, double> UR8ControllerQPIK::compute()
{
    if (is_mode_changed_)
    {
        is_mode_changed_ = false;
        control_start_time_ = sim_time_;

        q_init_ = q_;
        qdot_init_ = qdot_;
        link_ee_task_[ee_link_name_].setInit();

        q_desired_ = q_init_;
        qdot_desired_.setZero(dof_);
        link_ee_task_[ee_link_name_].setDesired();
        link_ee_task_[ee_link_name_].xdot_desired.setZero();
    }

    // ── Home: all joints to zero ──
    if (control_mode_ == "Home")
    {
        Eigen::VectorXd q_home = Eigen::VectorXd::Zero(dof_);

        q_desired_ = robot_controller_->moveJointPositionCubic(q_home,
                                                               Eigen::VectorXd::Zero(dof_),
                                                               q_init_,
                                                               qdot_init_,
                                                               sim_time_,
                                                               control_start_time_,
                                                               5.0);

        qdot_desired_ = robot_controller_->moveJointVelocityCubic(q_home,
                                                                  Eigen::VectorXd::Zero(dof_),
                                                                  q_init_,
                                                                  qdot_init_,
                                                                  sim_time_,
                                                                  control_start_time_,
                                                                  5.0);

        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── QPIK: task-space, base-frame Y+Z offset ──
    else if (control_mode_ == "QPIK")
    {
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += Eigen::Vector3d(0.0, 0.15, 0.15);

        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame translation: +Y (↑) ──
    else if (control_mode_ == "QPIK_Up")
    {
        const Eigen::Matrix3d R_ee = link_ee_task_[ee_link_name_].x_init.rotation();
        const Eigen::Vector3d offset_world = R_ee * Eigen::Vector3d(0.0, 0.05, 0.0);
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += offset_world;
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame translation: -Y (↓) ──
    else if (control_mode_ == "QPIK_Down")
    {
        const Eigen::Matrix3d R_ee = link_ee_task_[ee_link_name_].x_init.rotation();
        const Eigen::Vector3d offset_world = R_ee * Eigen::Vector3d(0.0, -0.05, 0.0);
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += offset_world;
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame translation: -X (←) ──
    else if (control_mode_ == "QPIK_Left")
    {
        const Eigen::Matrix3d R_ee = link_ee_task_[ee_link_name_].x_init.rotation();
        const Eigen::Vector3d offset_world = R_ee * Eigen::Vector3d(-0.05, 0.0, 0.0);
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += offset_world;
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame translation: +X (→) ──
    else if (control_mode_ == "QPIK_Right")
    {
        const Eigen::Matrix3d R_ee = link_ee_task_[ee_link_name_].x_init.rotation();
        const Eigen::Vector3d offset_world = R_ee * Eigen::Vector3d(0.05, 0.0, 0.0);
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += offset_world;
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame rotation: +X (W) ──
    else if (control_mode_ == "QPIK_Pitch_Forward")
    {
        const double angle = 5.0 * M_PI / 180.0;
        const Eigen::AngleAxisd delta_rot(angle, Eigen::Vector3d::UnitX());
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() =
            link_ee_task_[ee_link_name_].x_init.linear() * delta_rot.toRotationMatrix();
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame rotation: -X (S) ──
    else if (control_mode_ == "QPIK_Pitch_Anti")
    {
        const double angle = -5.0 * M_PI / 180.0;
        const Eigen::AngleAxisd delta_rot(angle, Eigen::Vector3d::UnitX());
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() =
            link_ee_task_[ee_link_name_].x_init.linear() * delta_rot.toRotationMatrix();
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame rotation: -Z (A) ──
    else if (control_mode_ == "QPIK_Roller_Anti")
    {
        const double angle = -5.0 * M_PI / 180.0;
        const Eigen::AngleAxisd delta_rot(angle, Eigen::Vector3d::UnitZ());
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() =
            link_ee_task_[ee_link_name_].x_init.linear() * delta_rot.toRotationMatrix();
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── EE-frame rotation: +Z (D) ──
    else if (control_mode_ == "QPIK_Roller_Forward")
    {
        const double angle = 5.0 * M_PI / 180.0;
        const Eigen::AngleAxisd delta_rot(angle, Eigen::Vector3d::UnitZ());
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() =
            link_ee_task_[ee_link_name_].x_init.linear() * delta_rot.toRotationMatrix();
        robot_controller_->QPIKCubic(link_ee_task_, 1.5, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── Gravity Compensation ──
    else if (control_mode_ == "Gravity Compensation")
    {
        tau_desired_ = robot_data_->getGravity();
    }
    // ── Pose Knife: 8-joint target pose (joint-space cubic) ──
    else if (control_mode_ == "Pose Knife")
    {
        Eigen::Matrix<double, 10, 1> q_knife;
        q_knife << 0.0, 0.1908, -0.7286, 0.1513, -2.8971, 1.995, -0.399, 0.1561, -0.1262, 0.0145;

        q_desired_ = robot_controller_->moveJointPositionCubic(q_knife,
                                                               Eigen::VectorXd::Zero(dof_),
                                                               q_init_,
                                                               qdot_init_,
                                                               sim_time_,
                                                               control_start_time_,
                                                               5.0);

        qdot_desired_ = robot_controller_->moveJointVelocityCubic(q_knife,
                                                                  Eigen::VectorXd::Zero(dof_),
                                                                  q_init_,
                                                                  qdot_init_,
                                                                  sim_time_,
                                                                  control_start_time_,
                                                                  5.0);

        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    // ── Gravity Compensation W QPID ──
    else if (control_mode_ == "Gravity Compensation W QPID")
    {
        link_ee_task_[ee_link_name_].xddot_desired.setZero();
        robot_controller_->QPID(link_ee_task_, tau_desired_);
    }

    std::unordered_map<std::string, double> ctrl_dict;
    ctrl_dict.reserve(dof_);
    for (size_t i = 0; i < dof_; ++i)
    {
        ctrl_dict.emplace(joint_names_[i], tau_desired_(i));
    }
    return ctrl_dict;
}

void UR8ControllerQPIK::setMode(const std::string& control_mode)
{
    is_mode_changed_ = true;
    control_mode_ = control_mode;
    std::cout << "Control Mode Changed: " << control_mode_ << std::endl;
}

void UR8ControllerQPIK::startKeyListener_()
{
    tty_ok_ = ::isatty(STDIN_FILENO);
    if (!tty_ok_)
    {
        std::cout << "[UR8Controller] stdin is not a TTY; keyboard control disabled.\n";
        return;
    }
    setRawMode_();
    stop_key_ = false;
    key_thread_ = std::thread(&UR8ControllerQPIK::keyLoop_, this);

    std::cout << "[UR8Controller] Keyboard:\n"
              << "  [1] Home  [2] QPIK  [3] Gravity Comp  [4] Gravity Comp W QPID\n"
              << "  [ArrowUp] EE +Y    [ArrowDown] EE -Y    [ArrowLeft] EE -X    [ArrowRight] EE +X\n"
              << "  [W] EE +X rot      [S] EE -X rot        [A] EE -Z rot        [D] EE +Z rot\n"
              << "  [Q] Quit\n";
}

void UR8ControllerQPIK::stopKeyListener_()
{
    if (!tty_ok_) return;
    stop_key_ = true;
    if (key_thread_.joinable()) key_thread_.join();
    restoreTerm_();
}

void UR8ControllerQPIK::setRawMode_()
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

void UR8ControllerQPIK::restoreTerm_()
{
    if (!tty_ok_) return;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_) == -1)
    {
        perror("tcsetattr restore");
    }
}

void UR8ControllerQPIK::keyLoop_()
{
    while (!stop_key_)
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 20 * 1000;

        int ret = ::select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds))
        {
            char buf[8];
            ssize_t nread = ::read(STDIN_FILENO, buf, sizeof(buf));
            for (ssize_t i = 0; i < nread; ++i)
            {
                char c = buf[i];
                if (c == '1')              setMode("Home");
                else if (c == '2')         setMode("QPIK");
                else if (c == '3')         setMode("Gravity Compensation");
                else if (c == '4')         setMode("Gravity Compensation W QPID");
                else if (c == '5')         setMode("Pose Knife");
                else if (c == '\x1b' && i + 2 < nread && buf[i+1] == '[')
                {
                    char code = buf[i+2];
                    if      (code == 'A')      setMode("QPIK_Up");
                    else if (code == 'B')      setMode("QPIK_Down");
                    else if (code == 'C')      setMode("QPIK_Right");
                    else if (code == 'D')      setMode("QPIK_Left");
                    i += 2;
                }
                else if (c == 'w' || c == 'W') setMode("QPIK_Pitch_Forward");
                else if (c == 's' || c == 'S') setMode("QPIK_Pitch_Anti");
                else if (c == 'a' || c == 'A') setMode("QPIK_Roller_Anti");
                else if (c == 'd' || c == 'D') setMode("QPIK_Roller_Forward");
                else if (c == 'q' || c == 'Q') stop_key_ = true;
            }
        }
    }
}
