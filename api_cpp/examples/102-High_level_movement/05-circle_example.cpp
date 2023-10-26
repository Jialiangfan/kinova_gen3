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

#include <BaseClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>



#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <fstream>



#include "utilities.h"

namespace k_api = Kinova::Api;

#define PORT 10000

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

// Create an event listener that will set the promise action event to the exit value
// Will set to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
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


// Create an event listener that will set the sent reference to the exit value
// Will set to either END or ABORT
// Read the value of returnAction until it is set
std::function<void(k_api::Base::ActionNotification)>
    create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            returnAction = action_event;
            break;
        default:
            break;

        }
    };
}
bool example_move_to_home_position(k_api::Base::BaseClient* base)
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
        return false;
    } 
    else 
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> promise;
        auto future = promise.get_future();
        auto notification_handle = base->OnNotificationActionTopic(
            create_action_event_listener_by_promise(promise),
            k_api::Common::NotificationOptions{}
        );

        base->ExecuteActionFromReference(action_handle);

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
}

// bool example_twist_command(k_api::Base::BaseClient* base)
// {
//     auto command = k_api::Base::TwistCommand();
//     command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    
    

//     std::cout << "Sending twist command for 5 seconds..." << std::endl;

//     auto twist = command.mutable_twist();
//     twist->set_linear_x(0.0f);
//     twist->set_linear_y(0.03f);
//     twist->set_linear_z(0.00f);
//     twist->set_angular_x(0.0f);
//     twist->set_angular_y(0.0f);
//     twist->set_angular_z(5.0f);
//     base->SendTwistCommand(command);

//     // Let time for twist to be executed
//     std::this_thread::sleep_for(std::chrono::milliseconds(5000));

//     std::cout << "Stopping robot ..." << std::endl;

//     // Make movement stop
//     base->Stop();
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//     return true;
// }


bool example_move_to_start_position(k_api::Base::BaseClient* base)
{
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

    const std::vector<double> angles= {34.7562,  315.987,   95.1719,    276.44,   320.026,    79.8111};   //gen3_lite
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


bool example_cartesian_action_movement(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic) 
{
    std::cout << "Starting Cartesian action movement ..." << std::endl;

    auto feedback = base_cyclic->RefreshFeedback();
    auto action = k_api::Base::Action();
    action.set_name("Example Cartesian action movement");
    action.set_application_data("");

    auto constrained_pose = action.mutable_reach_pose();
    auto pose = constrained_pose->mutable_target_pose();

    
    pose->set_x(feedback.base().tool_pose_x()-0.2);                // x (meters)
    // pose->set_y(feedback.base().tool_pose_y() - 0.1);          // y (meters)
    // pose->set_z(feedback.base().tool_pose_z() - 0.2);          // z (meters)

    pose->set_y(feedback.base().tool_pose_y() );          // y (meters)
    pose->set_z(feedback.base().tool_pose_z() - 0.2 );          // z (meters)

    

    // pose->set_theta_x(feedback.base().tool_pose_theta_x() );    // theta x (degrees)
    // pose->set_theta_y(feedback.base().tool_pose_theta_y());    // theta y (degrees)
    // pose->set_theta_z(feedback.base().tool_pose_theta_z());    // theta z (degrees)


    pose->set_theta_x(180);    // theta x (degrees)
    pose->set_theta_y(0);    // theta y (degrees)
    //pose->set_theta_z(feedback.base().tool_pose_theta_z());    // theta z (degrees)
    pose->set_theta_z(90);    // theta z (degrees)


    // Connect to notification action topic
    // (Reference alternative)
    // See angular examples for Promise alternative
    k_api::Base::ActionEvent event = k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT;
    auto reference_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_ref(event),
        k_api::Common::NotificationOptions()
    );

    std::cout << "Executing action" << std::endl;
    
         base->ExecuteAction(action);

    // Wait for reference value to be set
    // (Reference alternative)
    // See angular examples for Promise alternative
    // Set a timeout after 20s of wait
    const auto timeout = std::chrono::system_clock::now() + TIMEOUT_DURATION;
    while(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT &&
        std::chrono::system_clock::now() < timeout)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    base->Unsubscribe(reference_notification_handle);

    if(event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

    std::cout << "Cartesian movement completed" << std::endl;
    
    std::cout << "Reference value : " << k_api::Base::ActionEvent_Name(event) << std::endl;

    while ("ACTION_END" != k_api::Base::ActionEvent_Name(event)  )
    {
          base->ExecuteAction(action);
    }
    
   


    // 获取新的反馈
    auto new_feedback = base_cyclic->RefreshFeedback();

    // 打印新的反馈中的工具位置和角度
    std::cout << "Current tool position: "
              << "X: " << new_feedback.base().tool_pose_x() 
              << ", Y: " << new_feedback.base().tool_pose_y() 
              << ", Z: " << new_feedback.base().tool_pose_z() << std::endl;
    std::cout << "Current tool orientation: "
              << "Theta X: " << new_feedback.base().tool_pose_theta_x()
              << ", Theta Y: " << new_feedback.base().tool_pose_theta_y()
              << ", Theta Z: " << new_feedback.base().tool_pose_theta_z() << std::endl;

    // 等待3秒
    std::this_thread::sleep_for(std::chrono::seconds(3));

    return true;
}



// bool example_actuator_low_level_velocity_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
// {
//     bool return_status = true;

//     // Move arm to ready position
//     example_move_to_start_position(base);
    
//     k_api::BaseCyclic::Feedback base_feedback;

//     auto servoingMode = k_api::Base::ServoingModeInformation();

//     std::cout << "Initializing the arm to fetch joint angles" << std::endl;
//     try
//     {
//         // Fetch feedback
//         base_feedback = base_cyclic->RefreshFeedback();
        
//         int actuator_count = base->GetActuatorCount().count();

//         // Print each actuator's current position
//         for(int i = 0; i < actuator_count; i++)
//         {
//             std::cout << "Actuator " << i << " position: " << base_feedback.actuators(i).position() << std::endl;
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
 
//     // Wait for a bit
//     std::this_thread::sleep_for(std::chrono::milliseconds(2000));

//     return return_status;
// }


bool example_twist_circle(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    const float RADIUS = 0.15f;  // Radius of the circle, in meters
    const float SPEED = 0.2f;   // Speed of the movement, in m/s
    const float FREQUENCY = 40.0f; // Control frequency, in Hz
   
    const float DELTA_ANGLE = SPEED / RADIUS * (1.0f / FREQUENCY);  // Angle increment for each iteration
    std::ofstream real_data_file("/home/max/kortex/api_cpp/examples/103-Gen3_uart_bridge/real_data.txt");
    std::ofstream desired_data_file("/home/max/kortex/api_cpp/examples/103-Gen3_uart_bridge/desired_data.txt");

    float current_angle = 0.0f; // Starting angle三

    auto command = k_api::Base::TwistCommand();
    command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    //command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);
    

    std::cout << "Moving in circle with Cartesian velocity..." << std::endl;



    // Loop for a predefined duration or until a stop condition
    for (int i = 0; i < 360 / DELTA_ANGLE; ++i) 
    {
        // Calculate the circle coordinates for velocity (tangential to the circle)
        float vx = -RADIUS * sin(current_angle) * SPEED;
        float vy = RADIUS * cos(current_angle) * SPEED;

        auto twist = command.mutable_twist();
        twist->set_linear_x(vx);
        twist->set_linear_y(vy);
        twist->set_linear_z(0.0f); // Assuming no movement in Z direction
        twist->set_angular_x(0.0f);
        twist->set_angular_y(0.0f);
        twist->set_angular_z(0.0f); // No rotation

        base->SendTwistCommand(command);

        // 在循环内部：
        auto feedback = base_cyclic->RefreshFeedback();
        real_data_file << feedback.base().tool_pose_x() << " "
                    << feedback.base().tool_pose_y() << " "
                    << feedback.base().tool_pose_z() << " "
                    << feedback.base().tool_pose_theta_x() << " "
                    << feedback.base().tool_pose_theta_y() << " "
                    << feedback.base().tool_pose_theta_z() << std::endl;

        desired_data_file << (RADIUS * cos(current_angle)) << " "
                        << (RADIUS * sin(current_angle)) << " "
                        << 0.0 << " "
                        << 0.0 << " "  // x偏转角理想值
                        << 0.0 << " "  // y偏转角理想值
                        << 0.0 << std::endl;  // z偏转角理想值


        current_angle += DELTA_ANGLE;

    //     auto feedback = base_cyclic->RefreshFeedback();
    //    // 获取新的反馈
    //     auto new_feedback = base_cyclic->RefreshFeedback();
    //      // 打印新的反馈中的工具位置和角度
    //     std::cout  << ", Z: " << new_feedback.base().tool_pose_z() 
    //           << ", Theta Z: " << new_feedback.base().tool_pose_theta_z() << std::endl;

        // Sleep for a while before the next command
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0f / FREQUENCY)));
    }

    std::cout << "Stopping robot ..." << std::endl;

    real_data_file.close();
    desired_data_file.close();

    // Make movement stop
    base->Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return true;
}



// bool example_twist_command(k_api::Base::BaseClient* base)
// {
//     auto command = k_api::Base::TwistCommand();
//     command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
//     command.set_duration(0); // Unlimited time to execute 
//     long long deltaT = 0; //时间粒度
//     float R = 0.1f; //圆形半径

//     // loop control 
//     auto startTimer = std::chrono::high_resolution_clock::now();
//     while(true) // 这里我使用了一个无限循环，您可以根据需要添加退出条件
//     {
//         float a = R * sin(deltaT * M_PI / 180.0); 
//         float b = R * cos(deltaT * M_PI / 180.0); 

//         if (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - startTimer).count() > 1000)
//         {
//             startTimer = std::chrono::high_resolution_clock::now();
//             auto twist = command.mutable_twist();
//             twist->set_linear_x(a);
//             twist->set_linear_y(b);
//             twist->set_linear_z(0.00f);
//             twist->set_angular_x(0.00f);
//             twist->set_angular_y(0.00f);
//             twist->set_angular_z(0.0f);

//             base->SendTwistCommand(command);
//             ++deltaT;
//         }

//         std::cout << "Drawing circle with deltaT: " << deltaT << std::endl;
//     }

//     // Make movement stop
//     base->Stop();
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//     return true;
// }


// bool example_twist_command(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
// {
//     auto command = k_api::Base::TwistCommand();
//     command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
//     command.set_duration(0); // Unlimited time to execute 
//     long long deltaT = 0; //时间粒度
//     float R = 0.08f; //圆形半径
//     const float SPEED = 0.1f;
//     float current_angle = 0.0f; // 初始角度
//     const float ANGLE_INCREMENT = 3.0f;  // 增加每次循环中的角度增量

//     // Define the output files
//     std::ofstream real_data_file("/home/max/kortex/api_cpp/examples/103-Gen3_uart_bridge/real_data.txt");
//     std::ofstream desired_data_file("/home/max/kortex/api_cpp/examples/103-Gen3_uart_bridge/desired_data.txt");

//     // Get initial position of the end effector
//     auto initial_feedback = base_cyclic->RefreshFeedback();
//     float initial_x = initial_feedback.base().tool_pose_x();
//     float initial_y = initial_feedback.base().tool_pose_y();

//     // loop control 
//     auto startTimer = std::chrono::high_resolution_clock::now();
//     while(true) // 这里我使用了一个无限循环，您可以根据需要添加退出条件
//     {
//         float a = initial_x + R * sin(deltaT * M_PI / 180.0); 
//         float b = initial_y + R * cos(deltaT * M_PI / 180.0); 

//         if (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - startTimer).count() > 100000)
//         {
//             startTimer = std::chrono::high_resolution_clock::now();
//             cout << "开始运动" << endl;
            
//             current_angle += ANGLE_INCREMENT ;  // 使用ANGLE_INCREMENT来更新角度

//             float vx = -R * SPEED * sin(current_angle);
//             float vy = R * SPEED * cos(current_angle);
//             auto twist = command.mutable_twist();
//             twist->set_linear_x(vx);
//             twist->set_linear_y(vy);
//             twist->set_linear_z(0.00f);
//             twist->set_angular_x(0.00f);
//             twist->set_angular_y(0.00f);
//             twist->set_angular_z(0.0f);

//             base->SendTwistCommand(command);
//             cout << "运行结束" << endl;
            

//             // Get the real-time feedback from the robot
//             auto feedback = base_cyclic->RefreshFeedback();
            
//             // Write to the real_data file
//             real_data_file << feedback.base().tool_pose_x() << " "
//                            << feedback.base().tool_pose_y() << " "
//                            << feedback.base().tool_pose_z() << " "
//                            << feedback.base().tool_pose_theta_x() << " "
//                            << feedback.base().tool_pose_theta_y() << " "
//                            << feedback.base().tool_pose_theta_z() << "\n";

//             // Write to the desired_data file
//             desired_data_file << a << " "
//                               << b << " "
//                               << "0.00 " // Assuming Z is always 0
//                               << "0.00 " // Assuming theta_x is always 0
//                               << "0.00 " // Assuming theta_y is always 0
//                               << "0.00\n"; // Assuming theta_z is always 0
//          deltaT += ANGLE_INCREMENT;  // 使用ANGLE_INCREMENT来更新deltaT
           
//         }
      

//         std::cout << "Drawing circle with deltaT: " << deltaT << std::endl;
//     }

//     // Close the files
//     real_data_file.close();
//     desired_data_file.close();

//     // Make movement stop
//     base->Stop();
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//     return true;
// }


int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);
    // Example core

    //打印当前的关节

    // k_api::BaseCyclic::Feedback base_feedback;
    

    //  base_feedback = k_api::BaseCyclic::BaseCyclicClient::RefreshFeedback();

    // auto actuator_0 = base_feedback.actuators(0).position();
    // auto actuator_1 = base_feedback.actuators(1).position();
    // auto actuator_2 = base_feedback.actuators(2).position();
    // auto actuator_3 = base_feedback.actuators(3).position();
    // auto actuator_4 = base_feedback.actuators(4).position();
    // auto actuator_5 = base_feedback.actuators(5).position();

    // std::cout<< "actuator_0 = %f " << actuator_0 <<std::endl;
    // std::cout<< "actuator_1 = %f " << actuator_1 <<std::endl;
    // std::cout<< "actuator_2 = %f " << actuator_2<<std::endl;
    // std::cout<< "actuator_3 = %f " << actuator_3 <<std::endl;
    // std::cout<< "actuator_4 = %f " << actuator_4 <<std::endl;
    // std::cout<< "actuator_5 = %f " << actuator_5<<std::endl;

    

    bool success = true;

    //success &= example_move_to_home_position(base);

    //success &= example_twist_command(base);
    
    //success &= example_cartesian_action_movement(base, base_cyclic);


    // success &= example_move_to_start_position(base);
    //success &= example_twist_command(base,base_cyclic);

    // success &= example_twist_command(base);
    success &= example_twist_circle(base,base_cyclic);
    // Close API session
    session_manager->CloseSession();


    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete session_manager;
    delete router;
    delete transport;

    return success ? 0: 1;
}