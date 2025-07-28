
/* LiDAR header file - JP-0603 */


#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <vector>
#include <cstring>
#include <cstdint>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

// 配置與常數
#define REFRESH_BY_PSN
#define PRINT_DEGREE_RADIUS

//定義水平角查找表的大小
#define H_TABLE 1990

//光程補償修正開關
#define REVISE 0

//丟包率監控
#define TEST

// Debug flag
#define JPDEBUG 0
extern bool DEBUG;     //false; true;

// 基本配置常量
extern const std::string UDP_IP;
extern const int UDP_PORT;

// 數據包格式常量
extern const int HEADER_SIZE;
extern const int DATA_SIZE;
extern const int PACKET_SIZE;
extern const int POINTS_PER_PACKET;
extern const int TOTAL_POINTS_PER_LINE;
extern const uint8_t HEADER_MAGIC[4];
extern const int HEADER_START_OFFSET;
extern const int DATA_START_OFFSET;
extern const float START_READOUT;

// 定義每幀包含的掃描線數量
extern const int LINES_PER_FRAME;

// LiDAR參數常量
extern const float AZIMUTH_RESOLUTION;
extern const float ELEVATION_START_UPPER;
extern const float ELEVATION_START_LOWER;
extern const float ELEVATION_STEP;

// 數據包類型常量
extern const uint8_t PACKET_UPPER;
extern const uint8_t PACKET_LOWER;
extern const uint8_t ECHO_1ST;
extern const uint8_t ECHO_2ND;

// 回波選擇模式
enum EchoMode {
    ECHO_MODE_ALL = 0,    // 顯示所有回波
    ECHO_MODE_1ST = 1,    // 只顯示第一回波
    ECHO_MODE_2ND = 2     // 只顯示第二回波
};

//Table extern 宣告
extern const float vu_table[260];
extern const float vd_table[260];
extern const double h_lookup_table[H_TABLE][2];

// 定義點的結構
struct LidarPoint {
    float x, y, z;
    uint8_t intensity;
    float azimuth;
    float radius_in;
    float elevation;
    bool valid;
    uint8_t echo_num;
    float r, g, b;
};

// 定義一個數據包結構，用於在線程間傳遞數據
struct UdpPacket {
    std::vector<uint8_t> data;
    bool is_valid;
};

// 數據包緩衝隊列和相關同步原語
extern std::queue<UdpPacket> packet_queue;
extern std::mutex queue_mutex;
extern std::condition_variable queue_cv;
extern std::atomic<bool> running;

// 用於統計性能的變數
extern std::atomic<int> packet_counter;
extern std::atomic<int> frame_counter;
extern ros::Time performance_start_time;


// 函式宣告
void printInt(const char* str, int val);
void printFloat(const char* str, float val);
void print3Float(const char* str, float val1, float val2, float val3);
void print3Int(const char* str, int val1, int val2, int val3);
void print2(const char* str, int val1, int val2);

bool check_packet_header(const std::vector<uint8_t>& data);

double find_closest_value(double y);
void parse_udp_packet(const std::vector<uint8_t>& data, std::vector<LidarPoint>& points, bool& is_upper_packet, EchoMode echo_mode);
sensor_msgs::PointCloud2 create_point_cloud_msg(const std::vector<LidarPoint> &points);
void udp_receiver_thread();
void udp_to_pointcloud2(ros::Publisher& pub, EchoMode echo_mode);


//定義DIFO
#ifndef LIDAR_UDP_H
#define LIDAR_UDP_H

// 定義 DIFO 封包結構
struct DifoPacket {
    char header[8];
    uint8_t reserve1;
    uint8_t reserve2;
    uint32_t source_ip;
    uint32_t destination_ip;
    uint8_t mac_address[6];
    uint16_t msop_port;
    uint16_t difop_port;
    char pl_version[5];
    char ps_version[5];
    uint8_t reserve3[16];
    uint8_t echo_mode;
    uint8_t time_sync_mode;
    uint8_t time_sync_status;
    uint8_t timestamp[10];
    uint16_t voltage;
    uint8_t reserve4[67];
    uint8_t fault_status;
    uint8_t reserve5[119];
};

#endif // LIDAR_UDP_H
































