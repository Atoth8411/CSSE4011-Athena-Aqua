#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <thread>
#include <chrono>

int main() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(9999);
    inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);

    int16_t x1 = 10, x2 = 20, y1 = 30, y2 = 40;
    int16_t data[4] = {x1, x2, y1, y2};


    while(1) {
        sendto(sock, data, sizeof(data), 0, (sockaddr*)&server_addr, sizeof(server_addr));
        std::cout << "Sent: " << x1 << ", " << x2 << ", " << y1 << ", " << y2 << "\n";
        //add sleep here
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
