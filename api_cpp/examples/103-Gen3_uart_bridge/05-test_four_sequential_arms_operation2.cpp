#include <iostream>
#include <thread>
#include <vector>
#include <condition_variable>
#include <mutex>

#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>

#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <cmath> 
#include <chrono>

#include <string>
#include <math.h>

#include <KDetailedException.h>

#include <TransportClientUdp.h>
#include <google/protobuf/util/json_util.h>

#include <fstream>

namespace k_api = Kinova::Api;

const int PORT = 10000;
const int PORT_REAL_TIME = 10001;

std::condition_variable cv;
std::mutex mtx;
int arm_stopped_count = 0;


// void drawCircleFunction(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic) 
// {
//    const float RADIUS = 0.08f;  // 圆的半径，以米为单位
//     const float SPEED = 0.6f;   // 运动速度，以米/秒为单位
//     const float FREQUENCY = 230.0f; // 控制频率，以Hz为单位

//     const float DELTA_ANGLE = SPEED / RADIUS * (1.0f / FREQUENCY);  // 每次迭代的角度增量

//     std::ofstream real_data_file("/home/max/kortex/api_cpp/examples/103-Gen3_uart_bridge/real_data.txt");
//     std::ofstream desired_data_file("/home/max/kortex/api_cpp/examples/103-Gen3_uart_bridge/desired_data.txt");

//     float current_angle = 0.0f; // 初始角度

//     auto command = k_api::Base::TwistCommand();
//     command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);

//     // 获取机器人初始位置
//     auto feedback = base_cyclic->RefreshFeedback();
//     float start_x = feedback.base().tool_pose_x();
//     float start_y = feedback.base().tool_pose_y();
    
//     // 计算圆心位置
//     float circle_center_x = start_x - RADIUS * cos(current_angle);
//     float circle_center_y = start_y - RADIUS * sin(current_angle);

//     for (int i = 0; i < 360 / DELTA_ANGLE; ++i) 
//     {
//         // 计算圆上的速度坐标
//         float vx = -RADIUS * sin(current_angle) * SPEED;
//         float vy = RADIUS * cos(current_angle) * SPEED;

//         auto twist = command.mutable_twist();
//         twist->set_linear_x(vx);
//         twist->set_linear_y(vy);
//         twist->set_linear_z(0.0f);
//         twist->set_angular_x(0.0f);
//         twist->set_angular_y(0.0f);
//         twist->set_angular_z(0.0f);

//         base->SendTwistCommand(command);

//         // 获取机器人当前位置反馈
//         feedback = base_cyclic->RefreshFeedback();

//         real_data_file << feedback.base().tool_pose_x() << " "
//                     << feedback.base().tool_pose_y() << " "
//                     << feedback.base().tool_pose_z() << " "
//                     << feedback.base().tool_pose_theta_x() << " "
//                     << feedback.base().tool_pose_theta_y() << " "
//                     << feedback.base().tool_pose_theta_z() << std::endl;

//         desired_data_file << (circle_center_x + RADIUS * cos(current_angle)) << " "
//                           << (circle_center_y + RADIUS * sin(current_angle)) << " "
//                           << 0.0 << " "
//                           << 0.0 << " "
//                           << 0.0 << " "
//                           << 0.0 << std::endl;

//         current_angle += DELTA_ANGLE;

//         std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0f / FREQUENCY)));
//     }

//     real_data_file.close();
//     desired_data_file.close();

//     base->Stop();
//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));

// }

void drawCircleFunction(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic) 
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

    auto start_time = std::chrono::steady_clock::now();
    
    while (true) 
    {
        // 计算画圆已经花费的时间
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time);
        if (elapsed_time.count() >= 5)  // 如果超过5秒，跳出循环
            break;

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
}



void example_twist_circle(const std::string& ip_address, int arm_num) 
{
    // Error callback
    auto error_callback = [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); };

    // Create transport, router and connect
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    //transport->connect(ip_address, PORT);
      if(!transport->connect(ip_address, PORT))
    {
        std::cerr << "Failed to connect to " << ip_address << " on port " << PORT << std::endl;
        return;
    }

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    //transport_real_time->connect(ip_address, PORT_REAL_TIME);
      if(!transport_real_time->connect(ip_address, PORT_REAL_TIME))
    {
        std::cerr << "Failed to connect to " << ip_address << " on port " << PORT_REAL_TIME << std::endl;
        return;
    }




    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);

    // Wait for the previous arm to finish if not the first arm
    if (arm_num != 1)
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [arm_num] { return arm_stopped_count == arm_num - 1; });
    }

    // Your circle drawing code goes here
    // ...
    drawCircleFunction(base, base_cyclic);  // 调用画圆的函数
    std::cout << "Arm " << arm_num << " finished drawing circle." << std::endl;
    
    // Sleep for 5 seconds (simulate the drawing process for 5 seconds)
    //std::this_thread::sleep_for(std::chrono::seconds(5));

    // base->Stop();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    {
        std::lock_guard<std::mutex> lock(mtx);
        arm_stopped_count++;
         std::cout << "Arm " << arm_num << " stopped. Total stopped arms: " << arm_stopped_count << std::endl;
    }
    cv.notify_all();

    // Cleanup
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    delete base;
    delete base_cyclic;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}

int main() 
{
    std::vector<std::string> arm_ips = {
        "192.168.31.12",
        "192.168.31.14",
        //"192.168.31.16",
        //"192.168.31.17"
    };

    std::vector<std::thread> threads;

    int arm_num = 1;
    for (const auto& ip : arm_ips)
    {
        threads.emplace_back(example_twist_circle, ip, arm_num);
        arm_num++;
    }

    for (auto& t : threads)
    {
        t.join();
    }

    return 0;
}
