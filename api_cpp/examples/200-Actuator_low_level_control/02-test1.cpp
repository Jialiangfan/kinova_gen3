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
#include <fstream>

#include <thread>
#include <chrono>

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



#define DURATION 5             // Network timeout (seconds)


constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);


float velocity = 20.0f;         // Default velocity of the actuator (degrees per seconds)
float time_duration = DURATION; // Duration of the example (seconds)

// Waiting time during actions
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);

// Create closure to set finished to true after an END or an ABORT
std::function<void(k_api::Base::ActionNotification)> 
check_for_end_or_abort(bool& finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch(notification.action_event())
        {
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
    create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
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
int64_t GetTickUs()
{
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

/**************************
 * Example core functions *
 **************************/
void example_move_to_home_position(k_api::Base::BaseClient* base)
{
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
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    } 
    else 
    {
        bool action_finished = false; 
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options
        );

        base->ExecuteActionFromReference(action_handle);

        while(!action_finished)
        { 
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
}



// bool example_move_to_start_position(k_api::Base::BaseClient* base)
// {
//     // Make sure the arm is in Single Level Servoing before executing an Action
//     auto servoingMode = k_api::Base::ServoingModeInformation();
//     servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
//     base->SetServoingMode(servoingMode);
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
//     // Move arm to ready position
//     std::cout << "Moving the arm to the start position..." << std::endl;
//     auto constrained_joint_angles = k_api::Base::ConstrainedJointAngles();
//     auto joint_angles = constrained_joint_angles.mutable_joint_angles();

//     auto actuator_count = base->GetActuatorCount();
    
//     //设置起始位置为0，0，0，0，0，0
//     //const std::vector<double> angles(actuator_count.count(), 0.0);
//     //const std::vector<double> angles= {75.1088,  357.66,   106.82,    271.939,   293.039,   124.282};   //gen3_lite

//     const std::vector<double> angles= {39.41, 0, -90, 0, -90, 0};   //gen3_lite
//     //const std::vector<double> angles= {358.512,  17.4338,   178.775,    257.646,   0.603,   304.587, 87.20}; //gen3


//     for (size_t i = 0; i < angles.size(); ++i) 
//     {
//         auto joint_angle = joint_angles->add_joint_angles();
//         joint_angle->set_joint_identifier(i);
//         joint_angle->set_value(angles[i]);
//     }

//     // Connect to notification action topic
//     std::promise<k_api::Base::ActionEvent> promise;
//     auto future = promise.get_future();
//     auto notification_handle = base->OnNotificationActionTopic(
//         create_action_event_listener_by_promise(promise),
//         k_api::Common::NotificationOptions{}
//     );

//     base->PlayJointTrajectory(constrained_joint_angles);

//     // Wait for action to finish
//     const auto status = future.wait_for(TIMEOUT_DURATION);
//     base->Unsubscribe(notification_handle);

//     if(status != std::future_status::ready)
//     {
//         std::cout << "Timeout on action notification wait" << std::endl;
//         return false;
//     }

//     return true;
// }


bool example_move_to_start_position(k_api::Base::BaseClient* base)
{
    // ... [其他代码保持不变]
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Move arm to ready position
    std::cout << "Moving the arm to the start position..." << std::endl;
    auto constrained_joint_angles = k_api::Base::ConstrainedJointAngles();
    auto joint_angles = constrained_joint_angles.mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();
    
    //设置起始位置为0，0，0，0，0，0
    //const std::vector<double> angles(actuator_count.count(), 0.0);
    //const std::vector<double> angles= {75.1088,  357.66,   106.82,    271.939,   293.039,   124.282};   //gen3_lite

    const std::vector<double> angles= {39.41, 0, -90, 0, -90, 0};   //gen3_lite
    //const std::vector<double> angles= {358.512,  17.4338,   178.775,    257.646,   0.603,   304.587, 87.20}; //gen3


    for (size_t i = 0; i < angles.size(); ++i) 
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(angles[i]);
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

    if(status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

     return true;
}


// bool example_actuator_low_level_velocity_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
// {
//     bool return_status = true;

//     // Move arm to ready position
//     example_move_to_home_position(base);
    
//     k_api::BaseCyclic::Feedback base_feedback;
//     k_api::BaseCyclic::Command  base_command;

//     std::vector<float> commands;

//     auto servoingMode = k_api::Base::ServoingModeInformation();

//     int timer_count = 0;
//     int64_t now = 0;
//     int64_t last = 0;

//     int timeout = 0;

//     std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
//     try
//     {
//         // Set the base in low-level servoing mode
//         servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
//         base->SetServoingMode(servoingMode);
//         base_feedback = base_cyclic->RefreshFeedback();

//         int actuator_count = base->GetActuatorCount().count();

//         // Initialize each actuator to its current position
//         for(int i = 0; i < actuator_count; i++)
//         {
//             commands.push_back(base_feedback.actuators(i).position());
//             base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
//         }

//         // Define the callback function used in Refresh_callback
//         auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
//         {
//             // We are printing the data of the moving actuator just for the example purpose,
//             // avoid this in a real-time loop
//             std::string serialized_data;
//             google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
//             std::cout << serialized_data << std::endl << std::endl;
//         };

//         // Real-time loop
//         while(timer_count < (time_duration * 1000))
//         {
//             now = GetTickUs();
//             if(now - last > 1000)
//             {
//                 for(int i = 0; i < actuator_count; i++)
//                 {
//                     // Move only the last actuator to prevent collision
//         		    if(i == actuator_count - 1)
//         		    {
//                         commands[i] += (0.001f * velocity);
//                     	base_command.mutable_actuators(i)->set_position(fmod(commands[i], 360.0f));
//         		    }
//                 }

//                 try
//                 {
//                     base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
//                 }
//                 catch(...)
//                 {
//                     timeout++;
//                 }
                
//                 timer_count++;
//                 last = GetTickUs();
//             }
//         }
//     }
//     catch (k_api::KDetailedException& ex)
//     {
//         std::cout << "Kortex error: " << ex.what() << std::endl;
//         return_status = false;
//     }
//     catch (std::runtime_error& ex2)
//     {
//         std::cout << "Runtime error: " << ex2.what() << std::endl;
//         return_status = false;
//     }
 
//     // Set back the servoing mode to Single Level Servoing
//     servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
//     base->SetServoingMode(servoingMode);

//     // Wait for a bit
//     std::this_thread::sleep_for(std::chrono::milliseconds(2000));

//     return return_status;
// }

bool example_actuator_low_level_velocity_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, const std::string& filename, int frequency)
{
    bool return_status = true;

    // Move arm to ready position
    //example_move_to_home_position(base);
    
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    std::vector<float> commands;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;
    int64_t last_execution_time = 0;

    int timeout = 0;

    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;



    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cout << "Failed to open the file: " << filename << std::endl;
        return false;
    }

    try
    {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        int actuator_count = base->GetActuatorCount().count();

        // Initialize each actuator to its current position
        for(int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Define the callback function used in Refresh_callback
        auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
        {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            google::protobuf::util::MessageToJsonString(data.actuators(data.actuators_size() - 1), &serialized_data);
            std::cout << serialized_data << std::endl << std::endl;
        };

        std::string line;

       // Real-time loop
        // ... [其他代码保持不变]

// Real-time loop
        // while(std::getline(file, line))
        // {
        //     std::istringstream iss(line);
        //     float position_degree; // 直接读取的关节位置
        //     int joint_index = 0;

        //     while (iss >> position_degree && joint_index < actuator_count)
        //     {
        //         commands[joint_index] = position_degree; // 直接设置关节位置
        //         base_command.mutable_actuators(joint_index)->set_position(fmod(commands[joint_index], 360.0f));
        //         joint_index++;
        //     }

        //     try
        //     {
        //         base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
        //         now = GetTickUs();
        //         std::cout << "Time interval since last execution: " << (now - last_execution_time) / 1000.0 << " ms" << std::endl;
        //         last_execution_time = now;
        //     }
        //     catch(...)
        //     {
        //         timeout++;
        //     }

        //     // Sleep for the desired frequency
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frequency));
        // }

// ... [其他代码保持不变]

        // ... [其他代码保持不变]

        // Real-time loop
        while(std::getline(file, line))
        {
            std::istringstream iss(line);
            float position_degree; // 直接读取的关节位置
            int joint_index = 0;

            while (iss >> position_degree && joint_index < actuator_count)
            {
                commands[joint_index] = position_degree; // 直接设置关节位置
                base_command.mutable_actuators(joint_index)->set_position(fmod(commands[joint_index], 360.0f));
                joint_index++;
            }

            // Send the command
            base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);

            // Wait until all joints reach their target positions
            bool all_joints_in_position = false;
            while (!all_joints_in_position)
            {
                all_joints_in_position = true;
                base_feedback = base_cyclic->RefreshFeedback();
                for (int i = 0; i < actuator_count; i++)
                {
                    float error = fabs(base_feedback.actuators(i).position() - commands[i]);
                    if (error > 0.1) // Assuming 0.1 as the threshold
                    {
                        all_joints_in_position = false;
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Check every 10ms
            }
        }

// ... [其他代码保持不变]


    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
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




bool print_each_actuator_current_position(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    bool return_status = true;

    // Move arm to ready position
    //example_move_to_home_position(base);
    
    k_api::BaseCyclic::Feedback base_feedback;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    std::cout << "Initializing the arm to fetch joint angles" << std::endl;
    try
    {
        // Fetch feedback
        base_feedback = base_cyclic->RefreshFeedback();
        
        int actuator_count = base->GetActuatorCount().count();

        // Print each actuator's current position
        for(int i = 0; i < actuator_count; i++)
        {
            std::cout << "Actuator " << i << " position: " << base_feedback.actuators(i).position() << std::endl;
        }
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }
 
    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}




int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);

    // Example core

   
    auto isOk1 = example_move_to_start_position(base); 
    if (isOk1)
    {
        std::cout << "到位" << std::endl;
    }
    else{
            std::cout << "There has been an unexpected error in example_move_to_start_position() function." << std::endl;
    }

    print_each_actuator_current_position(base, base_cyclic);

    std::string response;
    do {
        std::cout << "是否继续运行? (yes/no): ";
        std::cin >> response;
    } while (response != "yes" && response != "no");

    if (response == "yes") 
    {
        std::string input_filename = "/home/max/kortex/api_cpp/examples/200-Actuator_low_level_control/xyh_test.txt";  //文件地址
        int frequency = 10;   
        if(isOk1)
        {
        auto isOk2 = example_actuator_low_level_velocity_control(base, base_cyclic,input_filename, frequency); 
            if (!isOk2)
            {
                std::cout << "There has been an unexpected error in example_actuator_low_level_velocity_control() function." << std::endl;
            }
        }
        } else {
            std::cout << "程序结束。" << std::endl;
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
