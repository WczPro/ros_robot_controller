// Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
// UR8 Controller — QPID (QP Inverse Dynamics) for task-space control
// QPID directly computes joint torques via QP: x_err → xddot_des → QP → τ
// Constraints: dynamics (M), torque limits, joint limits (CBF), collision (CBF)

#include "ur8_controller_qpid.hpp"
#include <sys/select.h>

#ifndef ROBOTS_DIRECTORY
#  error "ROBOTS_DIRECTORY is not defined."
#endif

UR8ControllerQPID::UR8ControllerQPID(const double dt)
: dt_(dt)
{
    const std::string urdf = std::string(ROBOTS_DIRECTORY) + "/ur8/" + "ur8.urdf";
    const std::string srdf = std::string(ROBOTS_DIRECTORY) + "/ur8/" + "ur8.srdf";

    robot_data_ = std::make_shared<drc::Manipulator::RobotData>(dt_, urdf, srdf);
    robot_controller_ = std::make_shared<drc::Manipulator::RobotController>(robot_data_);

    dof_ = robot_data_->getDof();

    joint_names_.clear();
    for (int i = 1; i <= 8; ++i)
        joint_names_.push_back("joint" + std::to_string(i));
    joint_names_.push_back("joint_grasp_left");
    joint_names_.push_back("joint_grasp_right");

    q_.setZero(dof_);
    qdot_.setZero(dof_);
    q_desired_.setZero(dof_);
    qdot_desired_.setZero(dof_);
    q_init_.setZero(dof_);
    qdot_init_.setZero(dof_);
    tau_desired_.setZero(dof_);

    link_ee_task_[ee_link_name_] = drc::TaskSpaceData::Zero();

    //第一组参数-----------------------------------------------------------------------//
    // // ── Joint PD gains (for Home / Pose Knife) ──
    // joint_kp_.setZero(dof_);
    // joint_kv_.setZero(dof_);
    // joint_kp_ <<  5000.0, 8000.0, 12000.0, 3000.0, 2500.0, 2000.0, 1000.0,  500.0, 200.0, 200.0;
    // joint_kv_ <<  8000.0, 8000.0, 12000.0, 2400.0, 2000.0, 1600.0,  800.0,  400.0, 160.0, 160.0;

    // // ── Task-space ID gains (for QPID) ──
    // task_id_kp_      <<  80.0,  80.0,  80.0, 200.0, 200.0, 200.0;   // high rotation stiffness
    // task_id_kv_      <<  18.0,  18.0,  18.0,  28.0,  28.0,  28.0;   // critical damping

    // // ── QPID gains: mass-proportional damping ──
    // qpid_tracking_   << 1000.0, 1000.0, 1000.0, 3000.0, 3000.0, 3000.0;   // rotation 3x priority
    
    //第二组参数-----------------------------------------------------------------------//
    // Joint PD gains
    joint_kp_.setZero(dof_); joint_kv_.setZero(dof_);
    joint_kp_ <<  5000.0, 8000.0, 12000.0, 3000.0, 2500.0, 2000.0, 1000.0,  500.0, 200.0, 200.0;
    joint_kv_ <<  8000.0, 8000.0, 12000.0, 2400.0, 2000.0, 1600.0,  800.0,  400.0, 160.0, 160.0;

    task_id_kp_      <<  120.0,  120.0,  120.0, 300.0, 300.0, 300.0;   // high rotation stiffness to prevent drift
    task_id_kv_      <<  22.0,  22.0,  22.0,  35.0,  35.0,  35.0;   // ζ≈1.0 critically damped

    //跟踪权重 5x，QP 更重视精度
    qpid_tracking_   << 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0;  
    
    qpid_vel_damping_.setZero(dof_);
    qpid_acc_damping_.setZero(dof_);

    // Read M diagonal for mass-proportional damping (same strategy as HQPID)
    {
        robot_data_->updateState(q_, qdot_);
        Eigen::MatrixXd M = robot_data_->getMassMatrix();
        for (int i = 0; i < dof_; ++i) {
            double m_ii = std::max(std::abs(M(i,i)), 1e-3);
            qpid_acc_damping_(i) = m_ii * 1.0;     // proportional to M_ii
            qpid_vel_damping_(i) = m_ii * 0.1;     // 1/10 of acc damping
        }
        std::cout << "[QPID] Mass-proportional damping: acc=" << qpid_acc_damping_.transpose() << std::endl;
    }

    robot_controller_->setJointGain(joint_kp_, joint_kv_);
    robot_controller_->setIKGain(task_ik_kp_);
    robot_controller_->setIDGain(task_id_kp_, task_id_kv_);
    robot_controller_->setQPIKGain(qpik_tracking_, qpik_vel_damping_, qpik_acc_damping_);
    robot_controller_->setQPIDGain(qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);

    std::cout << "info: \n" << robot_data_->getVerbose() << std::endl;

    startKeyListener_();
}

UR8ControllerQPID::~UR8ControllerQPID() { stopKeyListener_(); }

void UR8ControllerQPID::updateModel(const double current_time,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qpos_dict,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qvel_dict)
{
    sim_time_ = current_time;
    for (size_t i = 0; i < dof_; ++i) {
        const std::string& key = joint_names_[i];
        q_(i)    = qpos_dict.at(key)[0];
        qdot_(i) = qvel_dict.at(key)[0];
    }
    robot_data_->updateState(q_, qdot_);
    link_ee_task_[ee_link_name_].x    = robot_data_->getPose(ee_link_name_);
    link_ee_task_[ee_link_name_].xdot = robot_data_->getVelocity(ee_link_name_);
    link_ee_task_[ee_link_name_].current_time = sim_time_;

    // Print manipulability every 200 steps
    {
        static int mani_cnt = 0;
        if (++mani_cnt % 200 == 0) {
            auto mani = robot_data_->getManipulability(false, false, ee_link_name_);
            std::cout << "[QPID Manipulability] m=" << mani.manipulability
                      << "  (0=singular, >0.1=safe)" << std::endl;
        }
    }
}

std::unordered_map<std::string, double> UR8ControllerQPID::compute()
{
    // Track previous mode to detect repeated same-key presses
    static std::string prev_mode;
    static Eigen::Affine3d saved_x_init = Eigen::Affine3d::Identity();
    static Eigen::Vector3d accumulated_offset = Eigen::Vector3d::Zero();
    static double accumulated_angle_x = 0.0;  // for Pitch
    static double accumulated_angle_z = 0.0;  // for Roller
    static double saved_start_time = 0.0;
    bool is_repeat = false;
    bool mode_just_changed = is_mode_changed_;

    if (is_mode_changed_) {
        const bool is_trans = (control_mode_ == "QPID_Up" || control_mode_ == "QPID_Down" ||
                               control_mode_ == "QPID_Left" || control_mode_ == "QPID_Right" ||
                               control_mode_ == "QPID_Forward" || control_mode_ == "QPID_Anti");
        const bool is_rot = (control_mode_ == "QPID_Pitch_Forward" || control_mode_ == "QPID_Pitch_Anti" ||
                             control_mode_ == "QPID_Roller_Forward" || control_mode_ == "QPID_Roller_Anti");
        is_repeat = (is_trans || is_rot) && (control_mode_ == prev_mode);

        is_mode_changed_ = false;

        if (is_repeat) {
            control_start_time_ = saved_start_time;
            q_init_ = q_; qdot_init_ = qdot_;
            link_ee_task_[ee_link_name_].x_init = saved_x_init;
        } else {
            control_start_time_ = sim_time_;
            saved_start_time = sim_time_;
            accumulated_offset = Eigen::Vector3d::Zero();
            accumulated_angle_x = 0.0;
            accumulated_angle_z = 0.0;
            q_init_ = q_; qdot_init_ = qdot_;
            link_ee_task_[ee_link_name_].setInit();
            saved_x_init = link_ee_task_[ee_link_name_].x_init;
        }
        q_desired_ = q_init_;
        qdot_desired_.setZero(dof_);
        link_ee_task_[ee_link_name_].setDesired();
        link_ee_task_[ee_link_name_].xdot_desired.setZero();
    }
    if (mode_just_changed) {
        prev_mode = control_mode_;
    }

    // ── Joint-space modes ──
    if (control_mode_ == "Home") {
        Eigen::VectorXd q_home = Eigen::VectorXd::Zero(dof_);
        q_desired_ = robot_controller_->moveJointPositionCubic(
            q_home, Eigen::VectorXd::Zero(dof_),
            q_init_, qdot_init_, sim_time_, control_start_time_, 5.0);
        qdot_desired_ = robot_controller_->moveJointVelocityCubic(
            q_home, Eigen::VectorXd::Zero(dof_),
            q_init_, qdot_init_, sim_time_, control_start_time_, 5.0);
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    else if (control_mode_ == "Pose Knife") {
        Eigen::Matrix<double, 10, 1> q_knife;
        q_knife << 0.0, 0.1908, -0.7286, 0.1513, -2.8971, 1.995, -0.399, 0.1561, -0.1262, 0.0145;
        q_desired_ = robot_controller_->moveJointPositionCubic(
            q_knife, Eigen::VectorXd::Zero(dof_),
            q_init_, qdot_init_, sim_time_, control_start_time_, 5.0);
        qdot_desired_ = robot_controller_->moveJointVelocityCubic(
            q_knife, Eigen::VectorXd::Zero(dof_),
            q_init_, qdot_init_, sim_time_, control_start_time_, 5.0);
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }

    // ── Task-space modes — QPID (QP-based Inverse Dynamics) ──
    else if (control_mode_ == "QPID_Base") {
        link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += Eigen::Vector3d(0.0, 0.15, 0.15);
        robot_controller_->QPIDCubic(link_ee_task_, 2.0, tau_desired_);
    }
    // ── Translation modes (accumulate offset on repeat key) ──
    else if (control_mode_ == "QPID_Up" || control_mode_ == "QPID_Down" ||
             control_mode_ == "QPID_Left" || control_mode_ == "QPID_Right" ||
             control_mode_ == "QPID_Forward" || control_mode_ == "QPID_Anti")
    {
        Eigen::Vector3d step;
        if      (control_mode_ == "QPID_Up")    step = Eigen::Vector3d(0.0,  0.05, 0.0);
        else if (control_mode_ == "QPID_Down")  step = Eigen::Vector3d(0.0, -0.05, 0.0);
        else if (control_mode_ == "QPID_Left")  step = Eigen::Vector3d(-0.05, 0.0, 0.0);
        else if (control_mode_ == "QPID_Right") step = Eigen::Vector3d( 0.05, 0.0, 0.0);
        else if (control_mode_ == "QPID_Forward") step = Eigen::Vector3d(0.0, 0.0,  0.05);
        else /* QPID_Anti */                     step = Eigen::Vector3d(0.0, 0.0, -0.05);

        if (mode_just_changed) {
            if (is_repeat) accumulated_offset += saved_x_init.rotation() * step;
            else           accumulated_offset  = saved_x_init.rotation() * step;
        }
        link_ee_task_[ee_link_name_].x_desired = saved_x_init;
        link_ee_task_[ee_link_name_].x_desired.translation() += accumulated_offset;
        robot_controller_->QPIDCubic(link_ee_task_, 2.0, tau_desired_);
    }
    // ── Rotation modes (accumulate angle on repeat key) ──
    else if (control_mode_ == "QPID_Pitch_Forward" || control_mode_ == "QPID_Pitch_Anti" ||
             control_mode_ == "QPID_Roller_Forward" || control_mode_ == "QPID_Roller_Anti")
    {
        const double step_deg = 5.0 * M_PI / 180.0;
        if (mode_just_changed) {
            if      (control_mode_ == "QPID_Pitch_Forward")  { if (is_repeat) accumulated_angle_x += step_deg;  else accumulated_angle_x =  step_deg; }
            else if (control_mode_ == "QPID_Pitch_Anti")     { if (is_repeat) accumulated_angle_x -= step_deg;  else accumulated_angle_x = -step_deg; }
            else if (control_mode_ == "QPID_Roller_Forward") { if (is_repeat) accumulated_angle_z += step_deg;  else accumulated_angle_z =  step_deg; }
            else /* QPID_Roller_Anti */                      { if (is_repeat) accumulated_angle_z -= step_deg;  else accumulated_angle_z = -step_deg; }
        }
        Eigen::Matrix3d R_target = saved_x_init.linear()
            * Eigen::AngleAxisd(accumulated_angle_x, Eigen::Vector3d::UnitX()).toRotationMatrix()
            * Eigen::AngleAxisd(accumulated_angle_z, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        link_ee_task_[ee_link_name_].x_desired = saved_x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() = R_target;
        link_ee_task_[ee_link_name_].x_desired.translation() += accumulated_offset;
        robot_controller_->QPIDCubic(link_ee_task_, 2.0, tau_desired_);
    }

    // ── Gravity compensation modes ──
    else if (control_mode_ == "Gravity Compensation") {
        tau_desired_ = robot_data_->getGravity();
    }
    else if (control_mode_ == "Gravity Compensation W QPID") {
        link_ee_task_[ee_link_name_].xddot_desired.setZero();
        robot_controller_->QPID(link_ee_task_, tau_desired_);
    }

    std::unordered_map<std::string, double> ctrl_dict;
    ctrl_dict.reserve(dof_);
    for (size_t i = 0; i < dof_; ++i)
        ctrl_dict.emplace(joint_names_[i], tau_desired_(i));
    return ctrl_dict;
}

void UR8ControllerQPID::setMode(const std::string& control_mode) {
    is_mode_changed_ = true;
    control_mode_ = control_mode;
    std::cout << "Control Mode Changed: " << control_mode_ << std::endl;
}

// ── Keyboard ──
void UR8ControllerQPID::startKeyListener_() {
    tty_ok_ = ::isatty(STDIN_FILENO);
    if (!tty_ok_) { std::cout << "[UR8 QPID] stdin not a TTY\n"; return; }
    setRawMode_();
    stop_key_ = false;
    key_thread_ = std::thread(&UR8ControllerQPID::keyLoop_, this);
    std::cout << "[UR8 QPID] Keyboard: [1]Home [5]PoseKnife [3]Grav [4]Grav+QPID\n"
              << "  [Arrows] EE translate  [WASD] EE rotate  [Q]Quit\n";
}
void UR8ControllerQPID::stopKeyListener_() {
    if (!tty_ok_) return;
    stop_key_ = true;
    if (key_thread_.joinable()) key_thread_.join();
    restoreTerm_();
}
void UR8ControllerQPID::setRawMode_() {
    if (!tty_ok_) return;
    struct termios raw;
    tcgetattr(STDIN_FILENO, &orig_term_);
    raw = orig_term_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}
void UR8ControllerQPID::restoreTerm_() {
    if (!tty_ok_) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_);
}
void UR8ControllerQPID::keyLoop_() {
    while (!stop_key_) {
        fd_set rfds; FD_ZERO(&rfds); FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 20*1000;
        int ret = ::select(STDIN_FILENO+1, &rfds, nullptr, nullptr, &tv);
        if (ret > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
            char buf[8];
            ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
            for (ssize_t i = 0; i < n; ++i) {
                char c = buf[i];
                if (c == '1')               setMode("Home");
                else if (c == '2')          setMode("QPID_Base");
                else if (c == '3')          setMode("Gravity Compensation");
                else if (c == '4')          setMode("Gravity Compensation W QPID");
                else if (c == '5')          setMode("Pose Knife");
                else if (c == '\x1b' && i+2 < n && buf[i+1]=='[') {
                    // Ctrl+Up/Down: \x1b[1;5A / \x1b[1;5B
                    if (i+5 < n && buf[i+2]=='1' && buf[i+3]==';' && buf[i+4]=='5') {
                        char code = buf[i+5]; i += 5;
                        if (code=='A') setMode("QPID_Forward");
                        else if (code=='B') setMode("QPID_Anti");
                    } else {
                        char code = buf[i+2]; i += 2;
                        if (code=='A') setMode("QPID_Up");
                        else if (code=='B') setMode("QPID_Down");
                        else if (code=='C') setMode("QPID_Right");
                        else if (code=='D') setMode("QPID_Left");
                    }
                }
                else if (c=='w'||c=='W') setMode("QPID_Pitch_Forward");
                else if (c=='s'||c=='S') setMode("QPID_Pitch_Anti");
                else if (c=='a'||c=='A') setMode("QPID_Roller_Anti");
                else if (c=='d'||c=='D') setMode("QPID_Roller_Forward");
                else if (c=='q'||c=='Q') stop_key_ = true;
            }
        }
    }
}