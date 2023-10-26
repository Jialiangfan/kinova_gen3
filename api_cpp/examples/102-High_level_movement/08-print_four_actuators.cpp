#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <thread>

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

void printActuatorAngle(const std::string& ip_address) {
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
    // NOTE: You might need to change the username and password if they are different
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin"); // Change to your actual username
    create_session_info.set_password("admin"); // Change to your actual password
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication for IP: " << ip_address << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);

    // Fetch feedback and print actuator angles
    auto base_feedback = base_cyclic->RefreshFeedback();
    int actuator_count = base->GetActuatorCount().count();
    for(int i = 0; i < actuator_count; i++)
    {
        std::cout << "IP: " << ip_address << " - Actuator " << i << " position: " << base_feedback.actuators(i).position() << std::endl;
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

int main(int argc, char **argv) {
    // Create threads for each IP address
    std::thread t1(printActuatorAngle, "192.168.31.12");
    std::thread t2(printActuatorAngle, "192.168.31.14");
    // std::thread t3(printActuatorAngle, "192.168.31.16");
    // std::thread t4(printActuatorAngle, "192.168.31.17");

    // Wait for all threads to complete
    t1.join();
    t2.join();
    // t3.join();
    // t4.join();

    return 0;
}
