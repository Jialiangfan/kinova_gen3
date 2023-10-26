
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
#include "utilities.h"
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
bool is_arm1_stopped = false;

void example_twist_circle(const std::string& ip_address, int arm_num) 
{
    // Error callback
    auto error_callback = [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); };

    // Create transport, router and connect
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(ip_address, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(ip_address, PORT_REAL_TIME);

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

    if(arm_num != 1)
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, []{return is_arm1_stopped;});
    }

    // Perform the circle drawing operation
    // ... (The circle drawing code, I'm omitting it here for brevity)
    


    // Sleep for 5 seconds (simulate the drawing process for 5 seconds)
    std::this_thread::sleep_for(std::chrono::seconds(5));

    base->Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    if(arm_num == 1)
    {
        is_arm1_stopped = true;
        cv.notify_all();
    }

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
        "192.168.31.16",
        "192.168.31.17"
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
