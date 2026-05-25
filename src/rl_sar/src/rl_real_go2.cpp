/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_go2.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

RL_Real::RL_Real(int argc, char **argv)
    : hs_sock_(-1),
      ss_sock_(-1),
      udp_running_(false),
      height_scan_received_(false),
      safety_status_received_(false),
      safety_status_safe_(false),
      safety_unsafe_streak_count_(0),
      safety_trigger_getdown_(false)
{
    bool wheel_mode = (argc > 2 && std::string(argv[2]) == "wheel");

    // read params from yaml
    this->ang_vel_axis = "body";
    this->robot_name = wheel_mode ? "go2w" : "go2";
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitLowCmd();
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    // create lowcmd publisher
    this->lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    this->lowcmd_publisher->InitChannel();
    // create lowstate subscriber
    this->lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    this->lowstate_subscriber->InitChannel(std::bind(&RL_Real::LowStateMessageHandler, this, std::placeholders::_1), 1);
    // create joystick subscriber
    this->joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    this->joystick_subscriber->InitChannel(std::bind(&RL_Real::JoystickHandler, this, std::placeholders::_1), 1);

    // init UDP receivers
    this->start_udp_receivers();

    // init MotionSwitcherClient
    this->msc.SetTimeout(10.0f);
    this->msc.Init();
    // Shut down motion control-related service
    while(this->QueryMotionStatus())
    {
        std::cout << "Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = this->msc.ReleaseMode();
        if (ret == 0)
        {
            std::cout << "ReleaseMode succeeded." << std::endl;
        }
        else
        {
            std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(1);
    }

    // loop
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    this->stop_udp_receivers();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

// ============================================================================
// UDP Receiver
// ============================================================================

void RL_Real::start_udp_receivers()
{
    udp_running_ = true;

    // Height scan UDP socket (port 9870, receives 187 floats = 748 bytes)
    hs_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (hs_sock_ >= 0)
    {
        int opt = 1;
        setsockopt(hs_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(UDP_HEIGHTSCAN_PORT);

        if (bind(hs_sock_, (struct sockaddr*)&addr, sizeof(addr)) == 0)
        {
            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 0;
            setsockopt(hs_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            hs_recv_thread_ = std::thread(&RL_Real::height_scan_udp_loop, this);
            std::cout << LOGGER::INFO << "[UDP] Height scan receiver started on port " << UDP_HEIGHTSCAN_PORT << std::endl;
        }
        else
        {
            std::cout << LOGGER::ERROR << "[UDP] Failed to bind height scan port " << UDP_HEIGHTSCAN_PORT << std::endl;
            close(hs_sock_);
            hs_sock_ = -1;
        }
    }

    // Safety status UDP socket (port 9871, receives 1 byte: 0=unsafe, 1=safe)
    ss_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (ss_sock_ >= 0)
    {
        int opt = 1;
        setsockopt(ss_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(UDP_SAFETY_STATUS_PORT);

        if (bind(ss_sock_, (struct sockaddr*)&addr, sizeof(addr)) == 0)
        {
            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 0;
            setsockopt(ss_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            ss_recv_thread_ = std::thread(&RL_Real::safety_status_udp_loop, this);
            std::cout << LOGGER::INFO << "[UDP] Safety status receiver started on port " << UDP_SAFETY_STATUS_PORT << std::endl;
        }
        else
        {
            std::cout << LOGGER::ERROR << "[UDP] Failed to bind safety status port " << UDP_SAFETY_STATUS_PORT << std::endl;
            close(ss_sock_);
            ss_sock_ = -1;
        }
    }
}

void RL_Real::stop_udp_receivers()
{
    udp_running_ = false;
    if (hs_recv_thread_.joinable()) hs_recv_thread_.join();
    if (ss_recv_thread_.joinable()) ss_recv_thread_.join();
    if (hs_sock_ >= 0) { close(hs_sock_); hs_sock_ = -1; }
    if (ss_sock_ >= 0) { close(ss_sock_); ss_sock_ = -1; }
}

void RL_Real::height_scan_udp_loop()
{
    const int expected_floats = 11 * 17;
    const int expected_bytes = expected_floats * sizeof(float);
    std::vector<char> buf(expected_bytes);

    while (udp_running_)
    {
        ssize_t n = recv(hs_sock_, buf.data(), buf.size(), 0);
        if (n < 0) continue; // timeout

        if (n != expected_bytes) continue; // wrong size, skip

        std::lock_guard<std::mutex> lock(height_scan_mutex_);
        height_scan_data_.resize(expected_floats);
        std::memcpy(height_scan_data_.data(), buf.data(), expected_bytes);
        height_scan_received_ = true;
    }
}

void RL_Real::safety_status_udp_loop()
{
    char buf[1];
    while (udp_running_)
    {
        ssize_t n = recv(ss_sock_, buf, sizeof(buf), 0);
        if (n < 0) continue; // timeout

        if (n == 1)
        {
            const bool is_safe = (buf[0] != 0);
            safety_status_safe_ = is_safe;
            safety_status_received_ = true;

            if (is_safe)
            {
                safety_unsafe_streak_count_ = 0;
            }
            else
            {
                const int streak = ++safety_unsafe_streak_count_;
                if (streak >= 10)
                {
                    safety_trigger_getdown_ = true;
                }
            }
        }
    }
}

// ============================================================================
// Control Logic
// ============================================================================

void RL_Real::GetState(RobotState<float> *state)
{
    if (this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::A);
    if (this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::B);
    if (this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::X);
    if (this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->unitree_joy.components.L1) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->unitree_joy.components.R1) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->unitree_joy.components.R1 && this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->unitree_joy.components.L1 && this->unitree_joy.components.R1) this->control.SetGamepad(Input::Gamepad::LB_RB);

    // Raw inputs
    float raw_x = this->joystick.ly();
    float raw_y = -this->joystick.lx();
    float raw_yaw = -this->joystick.rx();

    // Quantization parameters
    const float deadzone = 0.2f;
    const float mid_thres = 0.8f;
    const float v_low = 0.3f;
    const float v_high = 0.5f;

    // Quantize x
    if (raw_x > mid_thres) {
        raw_x = v_high;
    } else if (raw_x > deadzone) {
        raw_x = v_low;
    } else if (raw_x < -mid_thres) {
        raw_x = -v_high;
    } else if (raw_x < -deadzone) {
        raw_x = -v_low;
    } else {
        raw_x = 0.0f;
    }
    this->control.x = raw_x;

    // Quantize y
    if (raw_y > mid_thres) {
        raw_y = v_high;
    } else if (raw_y > deadzone) {
        raw_y = v_low;
    } else if (raw_y < -mid_thres) {
        raw_y = -v_high;
    } else if (raw_y < -deadzone) {
        raw_y = -v_low;
    } else {
        raw_y = 0.0f;
    }
    this->control.y = raw_y;

    // raw_yaw
    if (raw_yaw > 0.75) {
        raw_yaw = 0.75;
    }
    if (raw_yaw < -0.75) {
        raw_yaw = -0.75;
    }
    this->control.yaw = raw_yaw;

    if (!this->safety_status_safe_.load())
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }
    if (this->safety_trigger_getdown_.load())
    {
        this->fsm.RequestStateChange("RLFSMStateGetDown");
    }

    state->imu.quaternion[0] = this->unitree_low_state.imu_state().quaternion()[0];
    state->imu.quaternion[1] = this->unitree_low_state.imu_state().quaternion()[1];
    state->imu.quaternion[2] = this->unitree_low_state.imu_state().quaternion()[2];
    state->imu.quaternion[3] = this->unitree_low_state.imu_state().quaternion()[3];

    for (int i = 0; i < 3; ++i)
    {
        state->imu.gyroscope[i] = this->unitree_low_state.imu_state().gyroscope()[i];
    }
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        state->motor_state.q[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].q();
        state->motor_state.dq[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq();
        state->motor_state.tau_est[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_est();
    }
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->unitree_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].mode() = 0x01;
        this->unitree_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].q() = command->motor_command.q[i];
        this->unitree_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq() = command->motor_command.dq[i];
        this->unitree_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp() = command->motor_command.kp[i];
        this->unitree_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd() = command->motor_command.kd[i];
        this->unitree_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau() = command->motor_command.tau[i];
    }

    this->unitree_low_command.crc() = Crc32Core((uint32_t *)&unitree_low_command, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(unitree_low_command);
}

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);

    this->CheckStartupGuard();

    this->StateController(&this->robot_state, &this->robot_command);

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Real::CheckStartupGuard()
{
    static bool startup_guard_enabled = true;
    static auto last_check_tp = std::chrono::steady_clock::now();

    if (!startup_guard_enabled)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_check_tp).count() < 1)
    {
        return;
    }

    last_check_tp = now;
    const bool hs_ok = this->height_scan_received_.load();
    const bool ss_ok = this->safety_status_received_.load();

    if (hs_ok && ss_ok)
    {
        startup_guard_enabled = false;
        std::cout << LOGGER::INFO << "[StartupGuard] height_scan and safety_status are ready, guard disabled." << std::endl;
        return;
    }

    if (!hs_ok || !ss_ok)
    {
        std::cout << LOGGER::WARNING << "[StartupGuard] Missing UDP inputs. height_scan_received="
                  << hs_ok << ", safety_status_received=" << ss_ok << std::endl;

        if (this->fsm.current_state_ && this->fsm.current_state_->GetStateName() != "RLFSMStatePassive")
        {
            this->fsm.RequestStateChange("RLFSMStatePassive");
        }
    }
}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        this->obs.base_quat = this->robot_state.imu.quaternion;

        this->real_dof_pos = this->robot_state.motor_state.q;
        this->real_dof_vel = this->robot_state.motor_state.dq;
        this->remapJointData(this->real_dof_pos, this->params.Get<std::vector<int>>("remap_joint_real2policy"), this->obs.dof_pos);
        this->remapJointData(this->real_dof_vel, this->params.Get<std::vector<int>>("remap_joint_real2policy"), this->obs.dof_vel);

        // Get height scan from UDP receiver
        if (this->height_scan_received_.load())
        {
            std::lock_guard<std::mutex> lock(this->height_scan_mutex_);
            if (this->height_scan_data_.size() == 11 * 17)
            {
                this->obs.height_scan = this->height_scan_data_;
            }
            else
            {
                this->obs.height_scan = std::vector<float>(11 * 17, 0.3f);
            }
        }
        else
        {
            this->obs.height_scan = std::vector<float>(11 * 17, 0.3f);
            std::cout << "\n !!!! Warning: height_scan is not received via UDP !!!! " << std::endl;
        }

#ifdef PRINT_OBS_ORIGINAL
        std::cout << "-------------------------------------------------------------" << std::endl;
        std::cout << "ang_vel: " << this->obs.ang_vel << std::endl;
        std::cout << "commands: " << this->obs.commands << std::endl;
        std::cout << "base_quat: " << this->obs.base_quat << std::endl;
        std::cout << "dof_pos: " << this->obs.dof_pos << std::endl;
        std::cout << "dof_vel: " << this->obs.dof_vel << std::endl;
        std::cout << "actions: " << this->obs.actions << std::endl;
        std::cout << "height_scan: " << this->obs.height_scan << std::endl;
#endif

        this->obs.actions = this->Forward();
        this->remapJointData(this->obs.actions, this->params.Get<std::vector<int>>("remap_joint_policy2real"), this->processed_actions);
        this->ComputeOutput(this->processed_actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        if (this->height_scan_enable)
        {
            this->clipVector(this->obs.height_scan, this->params.Get<std::vector<float>>("height_scan_clip"));
            this->obs.height_scan *= this->params.Get<float>("height_scan_scale");
            this->history_obs.insert(this->history_obs.end(), this->obs.height_scan.begin(), this->obs.height_scan.end());
        }
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->unitree_low_state.motor_state()[i].q());
        this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motor_cmd()[i].q());
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    plt::pause(0.0001);
}

uint32_t RL_Real::Crc32Core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; ++i)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; ++bits)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
            {
                CRC32 ^= dwPolynomial;
            }
            xbit >>= 1;
        }
    }

    return CRC32;
}

void RL_Real::InitLowCmd()
{
    this->unitree_low_command.head()[0] = 0xFE;
    this->unitree_low_command.head()[1] = 0xEF;
    this->unitree_low_command.level_flag() = 0xFF;
    this->unitree_low_command.gpio() = 0;

    for (int i = 0; i < 20; ++i)
    {
        this->unitree_low_command.motor_cmd()[i].mode() = (0x01);
        this->unitree_low_command.motor_cmd()[i].q() = (PosStopF);
        this->unitree_low_command.motor_cmd()[i].kp() = (0);
        this->unitree_low_command.motor_cmd()[i].dq() = (VelStopF);
        this->unitree_low_command.motor_cmd()[i].kd() = (0);
        this->unitree_low_command.motor_cmd()[i].tau() = (0);
    }
}

int RL_Real::QueryMotionStatus()
{
    std::string robotForm, motionName;
    int motionStatus;
    int32_t ret = this->msc.CheckMode(robotForm, motionName);
    if (ret == 0)
    {
        std::cout << "CheckMode succeeded." << std::endl;
    }
    else
    {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if (motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = QueryServiceName(robotForm, motionName);
        std::cout << "Service: " << serviceName << " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

std::string RL_Real::QueryServiceName(std::string form, std::string name)
{
    if (form == "0")
    {
        if (name == "normal" )   return "sport_mode";
        if (name == "ai" )       return "ai_sport";
        if (name == "advanced" ) return "advanced_sport";
    }
    else
    {
        if (name == "ai-w" )     return "wheeled_sport(go2W)";
        if (name == "normal-w" ) return "wheeled_sport(b2W)";
    }
    return "";
}

void RL_Real::LowStateMessageHandler(const void *message)
{
    this->unitree_low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void RL_Real::JoystickHandler(const void *message)
{
    joystick = *(unitree_go::msg::dds_::WirelessController_ *)message;
    this->unitree_joy.value = joystick.keys();
}

volatile sig_atomic_t g_shutdown_requested = 0;
void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", shutting down..." << std::endl;
    g_shutdown_requested = 1;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " networkInterface [wheel]" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }

    signal(SIGINT, signalHandler);
    ChannelFactory::Instance()->Init(0, argv[1]);

    RL_Real rl_sar(argc, argv);
    while (!g_shutdown_requested) { sleep(1); }
    std::cout << LOGGER::INFO << "Exiting..." << std::endl;

    return 0;
}


