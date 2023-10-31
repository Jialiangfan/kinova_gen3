/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>

#include "utilities.h"

#if defined(_MSC_VER)
#include <Windows.h>
#else

#include <unistd.h>

#endif

#include <time.h>

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001

#define DURATION 10            // Network timeout (seconds)
#define NUM_PROCESS 4

float velocity = 20.0f;         // Default velocity of the actuator (degrees per seconds)
float time_duration = DURATION; // Duration of the example (seconds)
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

// Waiting time during actions
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);

// Create closure to set finished to true after an END or an ABORT
std::function<void(k_api::Base::ActionNotification)>
check_for_end_or_abort(bool &finished) {
    return [&finished](k_api::Base::ActionNotification notification) {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch (notification.action_event()) {
            case k_api::Base::ActionEvent::ACTION_ABORT:
            case k_api::Base::ActionEvent::ACTION_END:
                finished = true;
                break;
            default:
                break;
        }
    };
}

std::function<void(k_api::Base::ActionNotification)>
create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent> &finish_promise) {
    return [&finish_promise](k_api::Base::ActionNotification notification) {
        const auto action_event = notification.action_event();
        switch (action_event) {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs() {
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000)/frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

bool move_to_target_position(k_api::Base::BaseClient *base) {
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to the target position..." << std::endl;
    auto constrained_joint_angles = k_api::Base::ConstrainedJointAngles();
    auto joint_angles = constrained_joint_angles.mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();

//    const std::vector<float> angles{136, 309.5, 68.76, 269.98, 297.97, 86.39};
    const std::vector<float> angles{0.216738301215512, 0.121700141186891, 4.66239744039609, 1.49316681779883,
                                    4.95536191184157, 1.09293937175229};

    for (size_t i = 0; i < angles.size(); ++i) {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(angles[i] * 180 / M_PI);
    }

    // Connect to notification action topic
    std::promise<k_api::Base::ActionEvent> promise;
    auto future = promise.get_future();
    auto notification_handle = base->OnNotificationActionTopic(
            create_action_event_listener_by_promise(promise),
            k_api::Common::NotificationOptions{}
    );

    base->PlayJointTrajectory(constrained_joint_angles);

    // Wait for action to finish
    const auto status = future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(notification_handle);

    if (status != std::future_status::ready) {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

    return true;
}


/**************************
 * Example core functions *
 **************************/
void example_move_to_home_position(k_api::Base::BaseClient *base) {
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action: action_list.action_list()) {
        std::cout << "action name" << action.name() << std::endl;
        if (action.name() == "Home") {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    } else {
        bool action_finished = false;
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
                check_for_end_or_abort(action_finished),
                options
        );

        base->ExecuteActionFromReference(action_handle);

        while (!action_finished) {
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
}

bool example_actuator_low_level_velocity_control(k_api::Base::BaseClient *base,
                                                 k_api::BaseCyclic::BaseCyclicClient *base_cyclic) {
    bool return_status = true;

    std::cout << "move finished" << std::endl;
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;
    std::vector<float> commands;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    int timeout = 0;

    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
    try {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        int actuator_count = base->GetActuatorCount().count();

        // Initialize each actuator to its current position
        for (int i = 0; i < actuator_count; i++) {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Define the callback function used in Refresh_callback
        auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data) {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
            std::cout << serialized_data << std::endl << std::endl;
        };

        // Real-time loop
        while (timer_count < (time_duration * 1000)) {
            now = GetTickUs();
            if (now - last > 1000) {
                for (int i = 0; i < actuator_count; i++) {
                    // Move only the last actuator to prevent collision
                    if (i == actuator_count - 1) {
                        commands[i] += (0.001f * velocity);
                        base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
                    }
                }
                try {
                    base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
                }
                catch (...) {
                    timeout++;
                }

                timer_count++;
                last = GetTickUs();
            }
        }
    }
    catch (k_api::KDetailedException &ex) {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2) {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    return return_status;
}

std::array<float, 7> split_by_space(std::string str) {
    std::stringstream input(str);
    std::array<float, 7> res{};
    std::string result;
    int ii = 0;
    while (input >> result || ii < 7) {
        res[ii++] = stof(result);
    }
    return res;
}


std::vector<std::array<float, 7>> read_joint_velocity_from_file(const char *filename) {
    freopen(filename, "r", stdin);
    std::vector<std::array<float, 7>> res;
    std::string line;
    while (getline(std::cin, line)) {
        res.push_back(split_by_space(line));
    }
    std::cin.clear();
    return res;
}


bool
low_level_velocity_control_from_file(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic,
                                     Taskconfig config) {
    bool return_status = true;

    std::cout << "enter the control function" << std::endl;
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    std::vector<float> commands;

    auto servoingMode = k_api::Base::ServoingModeInformation();
    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    int timeout = 0;

    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
    try {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();
        int actuator_count = base->GetActuatorCount().count();
        // Initialize each actuator to its current position
        for (int i = 0; i < actuator_count; i++) {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }
        // Define the callback function used in Refresh_callback
        auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data) {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
            std::cout << serialized_data << std::endl << std::endl;
        };
        std::vector<std::array<float, 7>> joint_velocities = read_joint_velocity_from_file(
                config.joint_velocity_file);
        std::cout << "read finished, joint velocity size:" << joint_velocities.size() << std::endl;
        // Real-time loop
        int loops = 0;
        while (timer_count < (time_duration * 1000)) {
//            std::cout << "current joint velocity:" << std::endl;
//            for (int i = 0; i < 7; i++) {
//                std::cout << current_joint_velocity[i] * 180 / M_PI << " ";
//            }
//            std::cout << std::endl;
            now = GetTickUs();
            if (now - last > 1000) {
                std::array<float, 7> current_joint_velocity = joint_velocities.front();
                joint_velocities.erase(joint_velocities.begin());
                for (int i = 0; i < actuator_count; i++) {
                    // Move only the last actuator to prevent collision
                    commands[i] += (0.001f * current_joint_velocity[i] * 180 / M_PI);
//                    commands[i] = current_joint_velocity[i]*  180 / M_PI;
                    std::cout << "joint position current" << commands[i] << std::endl;
                    base_command.mutable_actuators(i)->set_position(commands[i]);
//                    base_command.mutable_actuators(i)->set_velocity(current_joint_velocity[i]* 180 / M_PI*1000);
                }
                try {
                    base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
                }
                catch (...) {
                    timeout++;
                }
                timer_count++;
                last = GetTickUs();
            }
            loops += 1;
        }
    }
    catch (k_api::KDetailedException &ex) {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2) {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}


void move_robot_to_initial_state(ExampleArgs robot_config) {
//  ################################### 创建session
    auto error_callback = [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); };


    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(robot_config.ip_address, PORT);
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(robot_config.ip_address, PORT_REAL_TIME);
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(robot_config.username);
    create_session_info.set_password(robot_config.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;
    auto base = new k_api::Base::BaseClient(router);
// clear fault
    try {
        base->ClearFaults();
    }
    catch (...) {
        std::cout << "Unable to clear robot faults" << std::endl;
    }


    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
//  ################################### 调用方法
    move_to_target_position(base);
    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();
    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}


void run_joint_velocity_from_file(ExampleArgs robot_config, Taskconfig task_config) {
//  ################################### 创建session
    auto error_callback = [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(robot_config.ip_address, PORT);
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(robot_config.ip_address, PORT_REAL_TIME);
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(robot_config.username);
    create_session_info.set_password(robot_config.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
//  ################################### 调用方法
    auto isOk = low_level_velocity_control_from_file(base, base_cyclic, task_config);
    if (!isOk) {
        std::cout << "There has been an unexpected error in example_cyclic_armbase() function." << std::endl;
    }
    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();
    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <move_to_initial: 0 or 1>" << std::endl;
        return -1;
    }

    ExampleArgs robot1 = {"192.168.2.24", "admin", "admin"};
//    运行程序前是否要回到初始位置
    bool move_to_start = std::stoi(argv[1]);

    if (move_to_start) {

//    move the robot to initial config
        move_robot_to_initial_state(robot1);

    }
    std::cout << "程序暂停5s，等待机械臂交出控制权" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "程序继续执行" << std::endl;



    Taskconfig config1 = {"/home/fjl2401/Downloads/circle-10.txt"};//new add
    run_joint_velocity_from_file(robot1,config1);
    // Example core
//    auto isOk = low_level_velocity_control_from_file(base, base_cyclic);
//    auto isOk =s example_actuator_low_level_velocity_control(base, base_cyclic);
//    if (!isOk) {
//        std::cout << "There has been an unexpected error in example_cyclic_armbase() function." << std::endl;
//    }


}
