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

#define DURATION 30            // Network timeout (seconds)
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

bool move_to_target_position(k_api::Base::BaseClient *base, string ip_address) {
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm  to the target position...  ip address: "<< ip_address << std::endl;
    auto constrained_joint_angles = k_api::Base::ConstrainedJointAngles();
    auto joint_angles = constrained_joint_angles.mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();

//    const std::vector<float> angles{136, 309.5, 68.76, 269.98, 297.97, 86.39};
    const std::vector<float> angles{0.216738301215512,0.121700141186891,4.66239744039609,1.49316681779883,4.95536191184157,1.09293937175229};

    for (size_t i = 0; i < angles.size(); ++i) {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(angles[i]*180/M_PI);
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



void move_robot_to_initial_state(ExampleArgs robot_config){
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
    pid_t pid = pthread_self();
    std::cout << "Creating sessions for communication, current pid: " << pid<< std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created, current pid: " << pid<< std::endl;
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
//  ################################### 调用方法
    move_to_target_position(base,robot_config.ip_address);
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

    ExampleArgs robot1={"192.168.2.21","admin","admin"};
    ExampleArgs robot2={"192.168.2.22","admin","admin"};
    ExampleArgs robot3={"192.168.2.23","admin","admin"};
    ExampleArgs robot4={"192.168.2.24","admin","admin"};

//    move the robot to initial config
    std::thread init_thread1(move_robot_to_initial_state,robot1);
    std::thread init_thread2(move_robot_to_initial_state,robot2);
    std::thread init_thread3(move_robot_to_initial_state,robot3);
    std::thread init_thread4(move_robot_to_initial_state,robot3);
    init_thread1.join();
    init_thread2.join();
    init_thread3.join();
    init_thread4.join();



    // Example core
//    auto isOk = low_level_velocity_control_from_file(base, base_cyclic);
//    auto isOk =s example_actuator_low_level_velocity_control(base, base_cyclic);
//    if (!isOk) {
//        std::cout << "There has been an unexpected error in example_cyclic_armbase() function." << std::endl;
//    }


}
