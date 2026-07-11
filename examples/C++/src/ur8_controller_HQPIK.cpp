// Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
// UR8 Controller — HQP IK (Hierarchical QP Inverse Kinematics)
// Tasks solved in strict priority: Level 1 = EE, Level 2+ = null-space

#include "ur8_controller_hqpik.hpp"
#include <sys/select.h>

#ifndef ROBOTS_DIRECTORY
#  error "ROBOTS_DIRECTORY is not defined."
#endif

UR8ControllerHQP::UR8ControllerHQP(const double dt)
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

    q_.setZero(dof_); qdot_.setZero(dof_);
    q_desired_.setZero(dof_); qdot_desired_.setZero(dof_);
    q_init_.setZero(dof_); qdot_init_.setZero(dof_);
    tau_desired_.setZero(dof_);
    link_ee_task_[ee_link_name_] = drc::TaskSpaceData::Zero();

    // ── Strategy (same as HQPID):
    // 1. Mass-proportional damping → avoid heavy joint abuse
    // 2. High rotation tracking weight → prevent orientation drift
    // 3. task_ik_kp_ for velocity-level IK (xdot_des = Kp * x_err)
    // 4. Cubic trajectory + HQPIK QP → qdot_des → q_des = q + qdot*dt → joint PD → tau

    // Joint PD gains (inner loop)
    joint_kp_.setZero(dof_); joint_kv_.setZero(dof_);
    joint_kp_ <<  5000.0, 8000.0, 12000.0, 3000.0, 2500.0, 2000.0, 1000.0,  500.0, 200.0, 200.0;
    joint_kv_ <<  8000.0, 8000.0, 12000.0, 2400.0, 2000.0, 1600.0,  800.0,  400.0, 160.0, 160.0;

    // Task-space IK gains (outer loop: xdot_des = Kp_ik * x_err)
    task_ik_kp_      <<  1.0,   1.0,   1.0,   2.0,   2.0,   2.0;   // ultra-slow: 0.05m error → 0.05 m/s, PD can track this
    // ID gains (not used by HQPIK, kept for gravity modes)
    task_id_kp_      <<  80.0,  80.0,  80.0, 150.0, 150.0, 150.0;
    task_id_kv_      <<  10.0,  10.0,  10.0,  20.0,  20.0,  20.0;

    // QPIK gains (for HQPIK velocity-level QP)
    // IMPORTANT: HQPIK QP is kinematic-only (no mass matrix!).
    // Damping must be LIGHT and UNIFORM — heavy damping blocks efficient kinematic paths.
    qpik_tracking_   << 100.0, 100.0, 100.0, 300.0, 300.0, 300.0;    // rotation 3x weight
    qpik_vel_damping_.setZero(dof_);
    qpik_vel_damping_ << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;  // uniform light
    qpik_acc_damping_.setZero(dof_);
    qpik_acc_damping_ << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;  // dt²=1e-6 amplifies this! Keep near-zero

    // QPID gains (not used by HQPIK, kept for gravity modes)
    qpid_tracking_   <<  20.0,  20.0,  20.0,  15.0,  15.0,  15.0;
    qpid_vel_damping_.setZero(dof_);
    qpid_vel_damping_ << 0.1, 0.1, 0.1, 0.05, 0.05, 0.03, 0.02, 0.01, 0.01, 0.01;
    qpid_acc_damping_.setZero(dof_);
    qpid_acc_damping_ << 0.05, 0.05, 0.05, 0.02, 0.02, 0.01, 0.01, 0.005, 0.005, 0.005;

    robot_controller_->setJointGain(joint_kp_, joint_kv_);
    robot_controller_->setIKGain(task_ik_kp_);
    robot_controller_->setIDGain(task_id_kp_, task_id_kv_);
    robot_controller_->setQPIKGain(qpik_tracking_, qpik_vel_damping_, qpik_acc_damping_);
    robot_controller_->setQPIDGain(qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);
    robot_controller_->setHQPIKGain(qpik_tracking_, qpik_vel_damping_, qpik_acc_damping_);

    std::cout << "info: \n" << robot_data_->getVerbose() << std::endl;
    startKeyListener_();
}

UR8ControllerHQP::~UR8ControllerHQP() { stopKeyListener_(); }

void UR8ControllerHQP::updateModel(const double t,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qpos,
                                const std::unordered_map<std::string, Eigen::VectorXd>& qvel)
{
    sim_time_ = t;
    for (size_t i = 0; i < dof_; ++i) {
        q_(i) = qpos.at(joint_names_[i])[0];
        qdot_(i) = qvel.at(joint_names_[i])[0];
    }
    robot_data_->updateState(q_, qdot_);
    link_ee_task_[ee_link_name_].x    = robot_data_->getPose(ee_link_name_);
    link_ee_task_[ee_link_name_].xdot = robot_data_->getVelocity(ee_link_name_);
    link_ee_task_[ee_link_name_].current_time = sim_time_;
}

std::unordered_map<std::string, double> UR8ControllerHQP::compute()
{
    if (is_mode_changed_) {
        is_mode_changed_ = false;
        control_start_time_ = sim_time_;
        q_init_ = q_; qdot_init_ = qdot_;
        link_ee_task_[ee_link_name_].setInit();
        q_desired_ = q_init_; qdot_desired_.setZero(dof_);
        link_ee_task_[ee_link_name_].setDesired();
        link_ee_task_[ee_link_name_].xdot_desired.setZero();
    }

    // ── Joint-space ──
    if (control_mode_ == "Home") {
        Eigen::VectorXd q_home = Eigen::VectorXd::Zero(dof_);
        q_desired_ = robot_controller_->moveJointPositionCubic(
            q_home, Eigen::VectorXd::Zero(dof_), q_init_, qdot_init_,
            sim_time_, control_start_time_, 5.0);
        qdot_desired_ = robot_controller_->moveJointVelocityCubic(
            q_home, Eigen::VectorXd::Zero(dof_), q_init_, qdot_init_,
            sim_time_, control_start_time_, 5.0);
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }
    else if (control_mode_ == "Pose Knife") {
        Eigen::Matrix<double, 10, 1> q_knife;
        q_knife << 0.0, 0.1908, -0.7286, 0.1513, -2.8971, 1.995, -0.399, 0.1561, -0.1262, 0.0145;
        q_desired_ = robot_controller_->moveJointPositionCubic(
            q_knife, Eigen::VectorXd::Zero(dof_), q_init_, qdot_init_,
            sim_time_, control_start_time_, 5.0);
        qdot_desired_ = robot_controller_->moveJointVelocityCubic(
            q_knife, Eigen::VectorXd::Zero(dof_), q_init_, qdot_init_,
            sim_time_, control_start_time_, 5.0);
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }

    // ── HQP IK task-space modes ──
    else if (control_mode_ == "HQPIK" ||
             control_mode_ == "HQPIK_Up"   || control_mode_ == "HQPIK_Down" ||
             control_mode_ == "HQPIK_Left" || control_mode_ == "HQPIK_Right" ||
             control_mode_ == "HQPIK_Pitch_Forward"  || control_mode_ == "HQPIK_Pitch_Anti" ||
             control_mode_ == "HQPIK_Roller_Forward" || control_mode_ == "HQPIK_Roller_Anti")
    {
        // Compute x_desired from current init + offset/rotation
        if (control_mode_ == "HQPIK") {
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.translation() += Eigen::Vector3d(0.0, 0.15, 0.15);
        }
        else if (control_mode_ == "HQPIK_Up") {
            auto R = link_ee_task_[ee_link_name_].x_init.rotation();
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.translation() += R * Eigen::Vector3d(0.0, 0.05, 0.0);
        }
        else if (control_mode_ == "HQPIK_Down") {
            auto R = link_ee_task_[ee_link_name_].x_init.rotation();
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.translation() += R * Eigen::Vector3d(0.0, -0.05, 0.0);
        }
        else if (control_mode_ == "HQPIK_Left") {
            auto R = link_ee_task_[ee_link_name_].x_init.rotation();
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.translation() += R * Eigen::Vector3d(-0.05, 0.0, 0.0);
        }
        else if (control_mode_ == "HQPIK_Right") {
            auto R = link_ee_task_[ee_link_name_].x_init.rotation();
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.translation() += R * Eigen::Vector3d(0.05, 0.0, 0.0);
        }
        else if (control_mode_ == "HQPIK_Pitch_Forward") {
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.linear() =
                link_ee_task_[ee_link_name_].x_init.linear()
                * Eigen::AngleAxisd(5.0*M_PI/180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
        }
        else if (control_mode_ == "HQPIK_Pitch_Anti") {
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.linear() =
                link_ee_task_[ee_link_name_].x_init.linear()
                * Eigen::AngleAxisd(-5.0*M_PI/180.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
        }
        else if (control_mode_ == "HQPIK_Roller_Forward") {
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.linear() =
                link_ee_task_[ee_link_name_].x_init.linear()
                * Eigen::AngleAxisd(5.0*M_PI/180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        }
        else if (control_mode_ == "HQPIK_Roller_Anti") {
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.linear() =
                link_ee_task_[ee_link_name_].x_init.linear()
                * Eigen::AngleAxisd(-5.0*M_PI/180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        }

        // Build HQP hierarchy
        std::vector<std::map<std::string, drc::TaskSpaceData>> hierarchy;
        hierarchy.push_back(link_ee_task_);  // Level 1: EE

        robot_controller_->HQPIKCubic(hierarchy, 3.0, qdot_desired_);
        q_desired_ = q_ + qdot_desired_ * dt_;

        Eigen::VectorXd tau_pd = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
        Eigen::VectorXd tau_grav = robot_data_->getGravity();
        tau_desired_ = tau_pd;  // PD only (no extra gravity — PD already compensates through position error)

        // ── DIAGNOSTIC: print key values to find the bug ──
        {
            static int diag_step = 0;
            ++diag_step;
            if (diag_step <= 5 || diag_step % 200 == 0) {
                Eigen::Vector6d x_err, xdot_err;
                DyrosMath::getTaskSpaceError(
                    link_ee_task_[ee_link_name_].x_desired,
                    link_ee_task_[ee_link_name_].xdot_desired,
                    link_ee_task_[ee_link_name_].x,
                    link_ee_task_[ee_link_name_].xdot,
                    x_err, xdot_err);
                std::cout << "[HQPIK diag#" << diag_step << "]"
                          << " pos_err=" << x_err.head<3>().transpose()
                          << " rot_err=" << x_err.tail<3>().transpose()
                          << "\n  q(0:3)=" << q_.head(4).transpose()
                          << "\n  qdot_des(0:3)=" << qdot_desired_.head(4).transpose()
                          << "\n  qdot(0:3)=" << qdot_.head(4).transpose()
                          << "\n  tau_pd(0:3)=" << tau_pd.head(4).transpose()
                          << "\n  tau_grav(0:3)=" << tau_grav.head(4).transpose()
                          << "\n  tau_total(0:3)=" << tau_desired_.head(4).transpose()
                          << std::endl;
            }
        }
    }

    // ── Gravity ──
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

void UR8ControllerHQP::setMode(const std::string& m) {
    is_mode_changed_ = true; control_mode_ = m;
    std::cout << "Control Mode Changed: " << m << std::endl;
}
void UR8ControllerHQP::startKeyListener_() {
    tty_ok_ = ::isatty(STDIN_FILENO);
    if (!tty_ok_) return;
    setRawMode_(); stop_key_ = false;
    key_thread_ = std::thread(&UR8ControllerHQP::keyLoop_, this);
    std::cout << "[UR8 HQP] Arrows=Translate WASD=Rotate 1=Home 5=Knife\n";
}
void UR8ControllerHQP::stopKeyListener_() {
    if (!tty_ok_) return; stop_key_ = true;
    if (key_thread_.joinable()) key_thread_.join(); restoreTerm_();
}
void UR8ControllerHQP::setRawMode_() {
    struct termios raw; tcgetattr(STDIN_FILENO, &orig_term_);
    raw = orig_term_; raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]=0; raw.c_cc[VTIME]=1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}
void UR8ControllerHQP::restoreTerm_() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_); }
void UR8ControllerHQP::keyLoop_() {
    while (!stop_key_) {
        fd_set rfds; FD_ZERO(&rfds); FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv{0, 20000};
        if (::select(STDIN_FILENO+1, &rfds, nullptr, nullptr, &tv) > 0
            && FD_ISSET(STDIN_FILENO, &rfds)) {
            char buf[8]; ssize_t n = ::read(STDIN_FILENO, buf, 8);
            for (ssize_t i = 0; i < n; ++i) {
                char c = buf[i];
                if (c=='1') setMode("Home");
                else if (c=='2') setMode("HQPIK");
                else if (c=='3') setMode("Gravity Compensation");
                else if (c=='4') setMode("Gravity Compensation W QPID");
                else if (c=='5') setMode("Pose Knife");
                else if (c=='\x1b' && i+2<n && buf[i+1]=='[') {
                    char o=buf[i+2]; i+=2;
                    if (o=='A') setMode("HQPIK_Up"); else if (o=='B') setMode("HQPIK_Down");
                    else if (o=='C') setMode("HQPIK_Right"); else if (o=='D') setMode("HQPIK_Left");
                }
                else if (c=='w'||c=='W') setMode("HQPIK_Pitch_Forward");
                else if (c=='s'||c=='S') setMode("HQPIK_Pitch_Anti");
                else if (c=='a'||c=='A') setMode("HQPIK_Roller_Anti");
                else if (c=='d'||c=='D') setMode("HQPIK_Roller_Forward");
                else if (c=='q'||c=='Q') stop_key_=true;
            }
        }
    }
}