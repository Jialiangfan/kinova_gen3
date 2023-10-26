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
#include "utilities.h"
#include <cmath> // 包含数学函数库
#include <chrono> // 包含时间相关的函数和数据类型

#include <fstream>  // 用于文件操作
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


bool example_twist_circle(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    const float RADIUS = 0.08f;  // 圆的半径，以米为单位
    const float SPEED = 0.6f;   // 运动速度，以米/秒为单位
    const float FREQUENCY = 230.0f; // 控制频率，以Hz为单位

    const float DELTA_ANGLE = SPEED / RADIUS * (1.0f / FREQUENCY);  // 每次迭代的角度增量

    std::ofstream real_data_file("/home/max/kortex/api_cpp/examples/103-Gen3_uart_bridge/real_data.txt");
    std::ofstream desired_data_file("/home/max/kortex/api_cpp/examples/103-Gen3_uart_bridge/desired_data.txt");

    float current_angle = 0.0f; // 初始角度

    auto command = k_api::Base::TwistCommand();
    command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);

    // 获取机器人初始位置
    auto feedback = base_cyclic->RefreshFeedback();
    float start_x = feedback.base().tool_pose_x();
    float start_y = feedback.base().tool_pose_y();
    
    // 计算圆心位置
    float circle_center_x = start_x - RADIUS * cos(current_angle);
    float circle_center_y = start_y - RADIUS * sin(current_angle);

    for (int i = 0; i < 360 / DELTA_ANGLE; ++i) 
    {
        // 计算圆上的速度坐标
        float vx = -RADIUS * sin(current_angle) * SPEED;
        float vy = RADIUS * cos(current_angle) * SPEED;

        auto twist = command.mutable_twist();
        twist->set_linear_x(vx);
        twist->set_linear_y(vy);
        twist->set_linear_z(0.0f);
        twist->set_angular_x(0.0f);
        twist->set_angular_y(0.0f);
        twist->set_angular_z(0.0f);

        base->SendTwistCommand(command);

        // 获取机器人当前位置反馈
        feedback = base_cyclic->RefreshFeedback();

        real_data_file << feedback.base().tool_pose_x() << " "
                    << feedback.base().tool_pose_y() << " "
                    << feedback.base().tool_pose_z() << " "
                    << feedback.base().tool_pose_theta_x() << " "
                    << feedback.base().tool_pose_theta_y() << " "
                    << feedback.base().tool_pose_theta_z() << std::endl;

        desired_data_file << (circle_center_x + RADIUS * cos(current_angle)) << " "
                          << (circle_center_y + RADIUS * sin(current_angle)) << " "
                          << 0.0 << " "
                          << 0.0 << " "
                          << 0.0 << " "
                          << 0.0 << std::endl;

        current_angle += DELTA_ANGLE;

        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0f / FREQUENCY)));
    }

    real_data_file.close();
    desired_data_file.close();

    base->Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    return true;
}




// // 圆形参数
// const float RADIUS = 0.05f;  // 圆的半径（单位：米）
// const float SPEED = 0.005f;   // 运动速度（单位：米/秒）
// const float K_PROPORTIONAL = 0.2f;  // 控制器的比例增益

// // 该函数返回带有漂移补偿的圆形运动的期望扭矩命令
// k_api::Base::TwistCommand circle_controller(float current_angle, const k_api::BaseCyclic::Feedback& feedback)
// {
//     k_api::Base::TwistCommand command; // 创建一个TwistCommand对象
//     command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL); // 设置参考帧为工具帧

//     float omega = SPEED / RADIUS;  // 计算期望的圆周上的角速度

//     // 计算圆上的期望位置
//     float desired_x = RADIUS * cos(current_angle);
//     float desired_y = RADIUS * sin(current_angle);

//     // 计算误差（当前位置与圆上期望位置之间的距离）
//     float error_x = feedback.base().tool_pose_x() - desired_x;
//     float error_y = feedback.base().tool_pose_y() - desired_y;
//     float error_distance = sqrt(error_x*error_x + error_y*error_y);
//     cout << error_distance << endl;

//     // 计算漂移的垂直补偿
//     float vR = -K_PROPORTIONAL * error_distance;

//     // 根据圆形运动公式计算速度
//     float vx = omega * (RADIUS * cos(current_angle)) + vR * sin(current_angle);
//     float vy = omega * (RADIUS * sin(current_angle)) - vR * cos(current_angle);

//     // 将速度设置到命令中
//     command.mutable_twist()->set_linear_x(vx);
//     command.mutable_twist()->set_linear_y(vy);
//     command.mutable_twist()->set_linear_z(0.0f);  // Z方向上无运动
//     command.mutable_twist()->set_angular_x(0.0f);
//     command.mutable_twist()->set_angular_y(0.0f);
//     command.mutable_twist()->set_angular_z(0.0f);  // 无旋转

//     return command; // 返回命令
// }

// bool example_twist_command(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
// {
//     // 当前圆上的角度
//     float current_angle = 0.0f;
//     float delta_angle = SPEED / RADIUS;  // 每次迭代的角度增量

//     std::cout << "使用Twist命令绘制圆形..." << std::endl;

//     // 无限循环，如有需要，可以添加退出循环的条件
//     while(true)
//     {
//         // 从机器人获取当前反馈
//         auto feedback = base_cyclic->RefreshFeedback();

//         // 使用circle_controller计算Twist命令
//         auto command = circle_controller(current_angle, feedback);

//         // 将命令发送到机器人
//         base->SendTwistCommand(command);

//         // 更新当前角度
//         current_angle += delta_angle;
//         if(current_angle > 2*M_PI)
//             current_angle -= 2*M_PI;

//         // 打印当前工具位置以供调试
//         std::cout << "工具位置 - X: " << feedback.base().tool_pose_x()
//                   << ", Y: " << feedback.base().tool_pose_y() << std::endl;

//         // 在下一个命令之前休眠一段短时间
//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//     }

//     return true; // 返回真
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
    //success &= example_twist_command(base,base_cyclic);  //使用修正参数k的圆
    success &= example_twist_circle(base,base_cyclic);   //第二个使用修正参数k的圆

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