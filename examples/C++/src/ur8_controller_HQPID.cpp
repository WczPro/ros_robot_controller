// Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
// UR8 Controller — HQP ID (Hierarchical QP Inverse Dynamics)

#include "ur8_controller_hqpid.hpp"
#include <sys/select.h>

#ifndef ROBOTS_DIRECTORY
#  error "ROBOTS_DIRECTORY is not defined."
#endif

UR8ControllerHQPID::UR8ControllerHQPID(const double dt)
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

    // HQPID 链 (ur8_HQPID 方向键):
    // task_ik_kp_=8   ×  HQPID_tracking_=10  → 两者乘起来放大跟踪
    // 如: 线性跟踪力 = 8×10 = 80，远超单看 Kp=8 的感觉

    // HQPID_vel_damping_=5.0  +  joint_kv_=8000~12000  → 两层阻尼叠加
    // QP 先压 qdot，PD 再用 Kv 追 — 7 个关节全被二次刹车

    // QPID 链 (ur8_qpid 方向键) — 完全独立:
    // 只用 task_id_kp/kv + qpid_*
    // task_ik_kp_ 和 HQPID_* 对 QPID 零影响

    // Home/Pose Knife (按键 1/5):
    // 只用 joint_kp/kv
    // 其他所有参数零影响

    // HQPID_Up 表现     →   改什么
    // ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    // 到位后来回抖      →   加大 HQPID_vel_damping_[对应关节]  (+0.1)
    // 到位太慢 / 到不了  →   加大 task_ik_kp_[线性]          (+2)
    // 到位后超调再回    →   加大 HQPID_vel_damping_[j2/j3]    (+0.1)
    // 软绵绵没力气      →   加大 HQPID_tracking_[线性]        (+5)
    // 平移方向不准确    →   减小 HQPID_vel_damping_[j1]       (-0.1)

    // 解读：
    // m 值	状态
    // > 1.0	灵活，所有方向易达
    // 0.1 ~ 1.0	正常
    // 0.02 ~ 0.1	⚠️ 接近奇异，CBF 开始约束
    // < 0.02	🔴 CBF 强约束，阻止进一步恶化
    // 0	奇异点（理论值，实际被 CBF 阻止）

    // //第一组参数-----------------------------------------------------------------------//
    // // Joint PD gains
    // joint_kp_.setZero(dof_); joint_kv_.setZero(dof_);
    // joint_kp_ <<  5000.0, 8000.0, 12000.0, 3000.0, 2500.0, 2000.0, 1000.0,  500.0, 200.0, 200.0;
    // joint_kv_ <<  8000.0, 8000.0, 12000.0, 2400.0, 2000.0, 1600.0,  800.0,  400.0, 160.0, 160.0;

    // task_ik_kp_      << 20.0,  20.0,  20.0,  19.0,  19.0,  19.0;   // not used by QPID
    // task_id_kp_      <<  80.0,  80.0,  80.0, 200.0, 200.0, 200.0;   // high rotation stiffness to prevent drift
    // task_id_kv_      <<  18.0,  18.0,  18.0,  28.0,  28.0,  28.0;   // ζ≈1.0 critically damped

    // qpik_tracking_   << 25.0,  25.0,  25.0,  20.0,  20.0,  20.0;
    // qpik_vel_damping_.setZero(dof_);
    // qpik_vel_damping_ << 150.0, 100.0, 150.0, 10.0, 20.0, 20.0, 5.11, 5.11, 5.11, 5.11;  // j3 heavily damped
    // qpik_acc_damping_.setZero(dof_);
    // qpik_acc_damping_ << 0.002, 0.001, 0.002, 0.0005, 0.0005, 0.0008, 0.001, 0.002, 0.002, 0.002;

    // //跟踪权重 5x，QP 更重视精度
    // //qpid_tracking_   << 1000.0, 1000.0, 1000.0, 3000.0, 3000.0, 3000.0;   // rotation 3x weight to prevent orientation drift
    // qpid_tracking_   << 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0; 

    //第二组参数-----------------------------------------------------------------------//
    // Joint PD gains
    joint_kp_.setZero(dof_); joint_kv_.setZero(dof_);
    joint_kp_ << 5000, 8000, 12000, 3000, 2500, 2000, 1000, 500, 200, 200;
    joint_kv_ << 8000, 8000, 12000, 2400, 2000, 1600,  800, 400, 160, 160;

    task_id_kp_      <<  120.0,  120.0,  120.0, 300.0, 300.0, 300.0;   // high rotation stiffness to prevent drift
    task_id_kv_      <<  22.0,  22.0,  22.0,  35.0,  35.0,  35.0;   // ζ≈1.0 critically damped

    //跟踪权重 5x，QP 更重视精度
    qpid_tracking_   << 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0; 
    qpid_vel_damping_.setZero(dof_);
    qpid_acc_damping_.setZero(dof_);

    // Read mass matrix diagonal to set mass-proportional damping
    // M_ii = effective inertia at joint i (j1: ~500kg, j8: ~0.005kg·m²)
    // Damping ∝ M_ii makes QP avoid heavy joints (like OSF's M⁻¹ weighting)
    {
        robot_data_->updateState(q_, qdot_);  // init state for mass matrix computation
        Eigen::MatrixXd M = robot_data_->getMassMatrix();
        for (int i = 0; i < dof_; ++i) {
            double m_ii = std::max(std::abs(M(i,i)), 1e-3);
            qpid_acc_damping_(i) = m_ii * 0.1;    // proportional to M_ii
            qpid_vel_damping_(i) = m_ii * 0.1;    // 1/10 of acc damping
        }
        std::cout << "[HQPID] Mass-proportional damping: acc=" << qpid_acc_damping_.transpose() << std::endl;
    }

    robot_controller_->setJointGain(joint_kp_, joint_kv_);
    robot_controller_->setIKGain(task_ik_kp_);
    robot_controller_->setIDGain(task_id_kp_, task_id_kv_);
    robot_controller_->setQPIKGain(qpik_tracking_, qpik_vel_damping_, qpik_acc_damping_);
    robot_controller_->setQPIDGain(qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);
    robot_controller_->setHQPIDGain(qpid_tracking_, qpid_vel_damping_, qpid_acc_damping_);

    std::cout << "info: \n" << robot_data_->getVerbose() << std::endl;
    startKeyListener_();
}

UR8ControllerHQPID::~UR8ControllerHQPID() { stopKeyListener_(); }

void UR8ControllerHQPID::updateModel(const double t,
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

std::unordered_map<std::string, double> UR8ControllerHQPID::compute()
{
    // Track previous mode to detect repeated same-key presses
    static std::string prev_mode;
    static Eigen::Affine3d saved_x_init = Eigen::Affine3d::Identity();
    static Eigen::Vector3d accumulated_offset = Eigen::Vector3d::Zero();
    static double accumulated_angle_x = 0.0;
    static double accumulated_angle_z = 0.0;
    static double saved_start_time = 0.0;
    bool is_repeat = false;
    bool mode_just_changed = is_mode_changed_;

    if (is_mode_changed_) {
        const bool is_trans = (control_mode_ == "HQPID_Up" || control_mode_ == "HQPID_Down" ||
                               control_mode_ == "HQPID_Left" || control_mode_ == "HQPID_Right" ||
                               control_mode_ == "HQPID_Forward" || control_mode_ == "HQPID_Anti");
        const bool is_rot = (control_mode_ == "HQPID_Pitch_Forward" || control_mode_ == "HQPID_Pitch_Anti" ||
                             control_mode_ == "HQPID_Roller_Forward" || control_mode_ == "HQPID_Roller_Anti");
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
        prev_mode = control_mode_;
        q_desired_ = q_init_; qdot_desired_.setZero(dof_);
        link_ee_task_[ee_link_name_].setDesired();
        link_ee_task_[ee_link_name_].xdot_desired.setZero();
    }
    if (mode_just_changed) {
        prev_mode = control_mode_;
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
        q_knife << 0.0, 0.1908, -0.7286, 0.0, -2.8971, 1.995, -0.399, 0.0, -0.1262, 0.0145;
       //q_knife << 0.0, 0.1908, -0.7286, 0.1513, -2.8971, 1.995, -0.399, 0.1561, -0.1262, 0.0145;
        q_desired_ = robot_controller_->moveJointPositionCubic(
            q_knife, Eigen::VectorXd::Zero(dof_), q_init_, qdot_init_,
            sim_time_, control_start_time_, 5.0);
        qdot_desired_ = robot_controller_->moveJointVelocityCubic(
            q_knife, Eigen::VectorXd::Zero(dof_), q_init_, qdot_init_,
            sim_time_, control_start_time_, 5.0);
        tau_desired_ = robot_controller_->moveJointTorqueStep(q_desired_, qdot_desired_, false);
    }

    // ──────────────────────────────────────────────────────
    //  HQP IK — strict priority hierarchy
    //  Level 1: EE pose tracking    (must satisfy first)
    //  Future levels: joint limits, posture, collision, ...
    // ──────────────────────────────────────────────────────
    else if (control_mode_ == "HQPID" ||
             control_mode_ == "HQPID_Up"   || control_mode_ == "HQPID_Down" ||
             control_mode_ == "HQPID_Left" || control_mode_ == "HQPID_Right" ||
             control_mode_ == "HQPID_Forward"  || control_mode_ == "HQPID_Anti" ||
             control_mode_ == "HQPID_Pitch_Forward"  || control_mode_ == "HQPID_Pitch_Anti" ||
             control_mode_ == "HQPID_Roller_Forward" || control_mode_ == "HQPID_Roller_Anti")
    {
        // Compute x_desired from current init + offset/rotation
        if (control_mode_ == "HQPID") {
            link_ee_task_[ee_link_name_].x_desired = link_ee_task_[ee_link_name_].x_init;
            link_ee_task_[ee_link_name_].x_desired.translation() += Eigen::Vector3d(0.0, 0.15, 0.15);
        }
        else if (control_mode_ == "HQPID_Up" || control_mode_ == "HQPID_Down" ||
                 control_mode_ == "HQPID_Left" || control_mode_ == "HQPID_Right" ||
                 control_mode_ == "HQPID_Forward" || control_mode_ == "HQPID_Anti")
        {
            // Translation in EE frame
            Eigen::Vector3d step;
            if      (control_mode_ == "HQPID_Up")    step = Eigen::Vector3d(0.0,  0.05, 0.0);
            else if (control_mode_ == "HQPID_Down")  step = Eigen::Vector3d(0.0, -0.05, 0.0);
            else if (control_mode_ == "HQPID_Left")  step = Eigen::Vector3d(-0.05, 0.0, 0.0);
            else if (control_mode_ == "HQPID_Right") step = Eigen::Vector3d( 0.05, 0.0, 0.0);
            else if (control_mode_ == "HQPID_Forward") step = Eigen::Vector3d(0.0, 0.0,  0.05);
            else /* HQPID_Anti */                     step = Eigen::Vector3d(0.0, 0.0, -0.05);

            // Only update offset on mode change (not every simulation step!)
            if (mode_just_changed) {  // only update offset on mode change, not every step
                if (is_repeat) {
                    accumulated_offset += saved_x_init.rotation() * step;
                } else {
                    accumulated_offset = saved_x_init.rotation() * step;
                }
            }
            link_ee_task_[ee_link_name_].x_desired = saved_x_init;
            link_ee_task_[ee_link_name_].x_desired.translation() += accumulated_offset;
        }
        // ── Rotation modes (accumulate angle on repeat key) ──
    else if (control_mode_ == "HQPID_Pitch_Forward" || control_mode_ == "HQPID_Pitch_Anti" ||
             control_mode_ == "HQPID_Roller_Forward" || control_mode_ == "HQPID_Roller_Anti")
    {
        const double step_deg = 5.0 * M_PI / 180.0;
        if (mode_just_changed) {
            if      (control_mode_ == "HQPID_Pitch_Forward")  { if (is_repeat) accumulated_angle_x += step_deg;  else accumulated_angle_x =  step_deg; }
            else if (control_mode_ == "HQPID_Pitch_Anti")     { if (is_repeat) accumulated_angle_x -= step_deg;  else accumulated_angle_x = -step_deg; }
            else if (control_mode_ == "HQPID_Roller_Forward") { if (is_repeat) accumulated_angle_z += step_deg;  else accumulated_angle_z =  step_deg; }
            else /* HQPID_Roller_Anti */                      { if (is_repeat) accumulated_angle_z -= step_deg;  else accumulated_angle_z = -step_deg; }
        }
        Eigen::Matrix3d R_target = saved_x_init.linear()
            * Eigen::AngleAxisd(accumulated_angle_x, Eigen::Vector3d::UnitX()).toRotationMatrix()
            * Eigen::AngleAxisd(accumulated_angle_z, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        link_ee_task_[ee_link_name_].x_desired = saved_x_init;
        link_ee_task_[ee_link_name_].x_desired.linear() = R_target;
        link_ee_task_[ee_link_name_].x_desired.translation() += accumulated_offset;
    }

        // Build HQP hierarchy
        std::vector<std::map<std::string, drc::TaskSpaceData>> hierarchy;
        hierarchy.push_back(link_ee_task_);  // Level 1: EE

        robot_controller_->HQPIDCubic(hierarchy, 1.5, tau_desired_);

        // Print manipulability every 200 steps
        {
            static int mani_cnt = 0;
            if (++mani_cnt % 200 == 0) {
                auto mani = robot_data_->getManipulability(false, false, ee_link_name_);
                std::cout << "[Manipulability] m=" << mani.manipulability
                          << "  (0=singular, >0.1=safe)" << std::endl;
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

void UR8ControllerHQPID::setMode(const std::string& m) {
    is_mode_changed_ = true; control_mode_ = m;
    std::cout << "Control Mode Changed: " << m << std::endl;
}
void UR8ControllerHQPID::startKeyListener_() {
    tty_ok_ = ::isatty(STDIN_FILENO);
    if (!tty_ok_) return;
    setRawMode_(); stop_key_ = false;
    key_thread_ = std::thread(&UR8ControllerHQPID::keyLoop_, this);
    std::cout << "[UR8 HQP] Arrows=Translate WASD=Rotate 1=Home 5=Knife\n";
}
void UR8ControllerHQPID::stopKeyListener_() {
    if (!tty_ok_) return; stop_key_ = true;
    if (key_thread_.joinable()) key_thread_.join(); restoreTerm_();
}
void UR8ControllerHQPID::setRawMode_() {
    struct termios raw; tcgetattr(STDIN_FILENO, &orig_term_);
    raw = orig_term_; raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]=0; raw.c_cc[VTIME]=1;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}
void UR8ControllerHQPID::restoreTerm_() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_); }
void UR8ControllerHQPID::keyLoop_() {
    while (!stop_key_) {
        fd_set rfds; FD_ZERO(&rfds); FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv{0, 20000};
        if (::select(STDIN_FILENO+1, &rfds, nullptr, nullptr, &tv) > 0
            && FD_ISSET(STDIN_FILENO, &rfds)) {
            char buf[8]; ssize_t n = ::read(STDIN_FILENO, buf, 8);
            for (ssize_t i = 0; i < n; ++i) {
                char c = buf[i];
                if (c=='1') setMode("Home");
                else if (c=='2') setMode("HQPID");
                else if (c=='3') setMode("Gravity Compensation");
                else if (c=='4') setMode("Gravity Compensation W QPID");
                else if (c=='5') setMode("Pose Knife");
                else if (c=='\x1b' && i+2<n && buf[i+1]=='[') {
                    // Ctrl+Up/Down: \x1b[1;5A / \x1b[1;5B
                    if (i+5<n && buf[i+2]=='1' && buf[i+3]==';' && buf[i+4]=='5') {
                        char o=buf[i+5]; i+=5;
                        if (o=='A') setMode("HQPID_Forward");
                        else if (o=='B') setMode("HQPID_Anti");
                    } else {
                        char o=buf[i+2]; i+=2;
                        if (o=='A') setMode("HQPID_Up"); else if (o=='B') setMode("HQPID_Down");
                        else if (o=='C') setMode("HQPID_Right"); else if (o=='D') setMode("HQPID_Left");
                    }
                }
                else if (c=='w'||c=='W') setMode("HQPID_Pitch_Forward");
                else if (c=='s'||c=='S') setMode("HQPID_Pitch_Anti");
                else if (c=='a'||c=='A') setMode("HQPID_Roller_Anti");
                else if (c=='d'||c=='D') setMode("HQPID_Roller_Forward");
                else if (c=='q'||c=='Q') stop_key_=true;
            }
        }
    }
}
