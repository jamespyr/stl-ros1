
/* Lidar main file - JP-0703*/
/* Notes:
1. Merge Github today checkout udp_recever_node.cpp
*/

#include "lidar_udp.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <thread>
#include <algorithm>
#include <vector>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <array>
#include <iomanip>
#include <cstdio>
#include <deque>
#include <condition_variable>

#ifdef TEST
// for FPS, PPS, Packet Loss Rate, Point Loss Rate
#include <numeric>
#include <fstream> 
#include <ctime>
#endif

using boost::asio::ip::udp;

//全域變數
bool DEBUG = false;

// 基本配置常量
const std::string UDP_IP = "192.168.48.10";  //發送端IP
const int UDP_PORT = 7000;  //傳輸主要資料

// DIFO 相關(預設)
#define DIFO_UDP_PORT 7788
#define DIFO_PACKET_SIZE 256

// 數據包格式常量
const int HEADER_SIZE = 32;
const int DATA_SIZE = 784;  // 260 points * 3 bytes per point
const int PACKET_SIZE = 816;  // HEADER_SIZE + DATA_SIZE
const int POINTS_PER_PACKET = 260;
const int TOTAL_POINTS_PER_LINE = 520;  // 上下半部總點數
const uint8_t HEADER_MAGIC[4] = {0x55, 0xaa, 0x5a, 0xa5};  
const int HEADER_START_OFFSET = 0;
const int DATA_START_OFFSET = 32; 
const float START_READOUT = 0.0;  //skip n ns after TRG_O set high

// 定義每幀包含的掃描線數量
const int LINES_PER_FRAME = 1990;  // 每幀包含的LiDAR掃描線數量

// LiDAR參數常量
const float AZIMUTH_RESOLUTION = 0.0439;  //0.055 for 2175 line   
const float ELEVATION_START_UPPER = 12.975;
const float ELEVATION_START_LOWER = -0.025;
const float ELEVATION_STEP = -0.05;

// 數據包類型常量
const uint8_t PACKET_UPPER = 0x10;
const uint8_t PACKET_LOWER = 0x20;
const uint8_t ECHO_1ST = 0x01;
const uint8_t ECHO_2ND = 0x02;

// 數據包緩衝隊列和相關同步原語
std::queue<UdpPacket> packet_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;
std::atomic<bool> running{true};

// 用於統計性能的變數
std::atomic<int> packet_counter{0};
std::atomic<int> frame_counter{0};
ros::Time performance_start_time;

// 定義每秒輸出點雲的數量 
std::deque<double> timestamp_buffer; // 每次接收到點雲時的時間（sec）
std::deque<int> point_count_buffer; // 每次接收到的點雲中點的數量
const int MAX_SECONDS = 60; // 分析的時間 （sec）

// ===== DIFO packet handling =====
// Utility: print MAC address as xx:xx:xx:xx:xx:xx
static void print_mac(const uint8_t mac[6]) {
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void print_difo_packet(const struct DifoPacket *pkt) {
    printf("DIFO Packet:\n");
    printf("  header: %.8s\n", pkt->header);
    printf("  reserve1: %u\n", pkt->reserve1);
    printf("  reserve2: %u\n", pkt->reserve2);
    struct in_addr src, dst;
    src.s_addr = pkt->source_ip;
    dst.s_addr = pkt->destination_ip;
    printf("  source_ip: %s\n", inet_ntoa(src));
    printf("  destination_ip: %s\n", inet_ntoa(dst));
    printf("  mac_address: ");
    print_mac(pkt->mac_address);
    printf("\n");
    printf("  msop_port: %u\n", ntohs(pkt->msop_port));
    printf("  difop_port: %u\n", ntohs(pkt->difop_port));
    printf("  pl_version: %.5s\n", pkt->pl_version);
    printf("  ps_version: %.5s\n", pkt->ps_version);
    printf("  echo_mode: %u\n", pkt->echo_mode);
    printf("  time_sync_mode: %u\n", pkt->time_sync_mode);
    printf("  time_sync_status: %u\n", pkt->time_sync_status);
    printf("  timestamp: ");
    for(int i = 0; i < 10; ++i) printf("%02X ", pkt->timestamp[i]);
    printf("\n");
    printf("  voltage: %u\n", ntohs(pkt->voltage));
    printf("  fault_status: %u\n", pkt->fault_status);
    
}

int parse_difo_packet(const uint8_t *buf, size_t len, struct DifoPacket *out_pkt) {
    if(len != DIFO_PACKET_SIZE) return -1;
    memcpy(out_pkt, buf, sizeof(struct DifoPacket));
    return 0;
}

// DIFO UDP receiver thread
void difo_udp_receiver_thread() {
    int sock;
    struct sockaddr_in addr;
    uint8_t buf[DIFO_PACKET_SIZE];
    struct DifoPacket pkt;

    if((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("DIFO socket");
        return;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(DIFO_UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if(bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("DIFO bind");
        close(sock);
        return;
    }

    printf("Listening for DIFO packets on UDP port %d...\n", DIFO_UDP_PORT);
    while(running) {
        ssize_t n = recv(sock, buf, sizeof(buf), 0);
        if(n == DIFO_PACKET_SIZE) {
            if(parse_difo_packet(buf, n, &pkt) == 0) {
                print_difo_packet(&pkt);
                // TODO: publish, store, or further process "pkt" as needed.
            }
        } else if (n > 0) {
            printf("Received unexpected DIFO packet size: %zd bytes\n", n);
        }
        // 可加 sleep/yield
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    close(sock);
}

// 檢查數據包頭部
bool check_packet_header(const std::vector<uint8_t>& data) {
    return (data.size() == PACKET_SIZE) && 
           (data[0] == HEADER_MAGIC[0]) && 
           (data[1] == HEADER_MAGIC[1]) && 
           (data[2] == HEADER_MAGIC[2]) && 
           (data[3] == HEADER_MAGIC[3]);
}

// 設定Pke_psn初始值
uint16_t last_pkt_psn = -1;
uint16_t pkt_psn = -1;

// 水平角查找函數：找到最接近的值
double find_closest_value(double y) {
    double closest_value = h_lookup_table[0][1];
    double min_diff = fabs(y - h_lookup_table[0][0]);

    for (int i = 1; i < H_TABLE; i++) {
        double diff = fabs(y - h_lookup_table[i][0]);
        if (diff < min_diff) {
            min_diff = diff;
            closest_value = h_lookup_table[i][1];
        }
    }

    return closest_value;
}

// UDP解析函數
void parse_udp_packet(const std::vector<uint8_t>& data, std::vector<LidarPoint>& points, bool& is_upper_packet, EchoMode echo_mode) {
    // 快速檢查數據包頭部
    if (!check_packet_header(data)) {
        return;
    }
    
    //Read packet psn
    last_pkt_psn = pkt_psn;
    pkt_psn = ((data[HEADER_START_OFFSET+5]<<8) | data[HEADER_START_OFFSET+4]); 

    // 解析回波序列和數據包類型
    uint8_t time_offset = data[DATA_START_OFFSET];
    uint8_t return_seq = data[DATA_START_OFFSET+1];
    uint8_t packet_type = return_seq & 0xF0;
    uint8_t echo_num = return_seq & 0x0F;

    float path_revise;
    float elevation;
    
    // 根據回波模式決定是否處理這個包
    if ((echo_mode == ECHO_MODE_1ST && echo_num != ECHO_1ST) || 
        (echo_mode == ECHO_MODE_2ND && echo_num != ECHO_2ND)) {
        return;
    }

    // 快速檢查包類型
    if (packet_type != PACKET_UPPER && packet_type != PACKET_LOWER) {
        return;
    }

    is_upper_packet = (packet_type == PACKET_UPPER);
    
    // 解析方位角
    int16_t azimuth_raw_in = (data[DATA_START_OFFSET+3] << 8) | data[DATA_START_OFFSET+2];
    float azimuth = float (azimuth_raw_in) / 100.0;    //Encoder校正完直接輸出角度值  

    if (REVISE){
        //增加光程修正部分,轉為公尺  JP-0407
        path_revise = find_closest_value(azimuth)/1000;
        //if(0.0f <= azimuth <= 1.0f){
        //   printf("[Debug msg] azimuth=%f, path_revise=%f\n", azimuth, path_revise);
        //}
    }
    
    // 預先計算方位角的三角函數值，避免重複計算
    float rad_azimuth = azimuth * M_PI / 180.0f;
    float cos_azimuth = cos(rad_azimuth);
    float sin_azimuth = sin(rad_azimuth);

    // 預先獲取起始仰角
    float elevation_start = is_upper_packet ? ELEVATION_START_UPPER : ELEVATION_START_LOWER;
    
    // 預分配需要的點數
    int expected_points = 0;
    switch(echo_mode) {
        case ECHO_MODE_ALL:
            expected_points = POINTS_PER_PACKET;
            break;
        case ECHO_MODE_1ST:
        case ECHO_MODE_2ND:
            if ((echo_mode == ECHO_MODE_1ST && echo_num == ECHO_1ST) ||
                (echo_mode == ECHO_MODE_2ND && echo_num == ECHO_2ND)) {
                expected_points = POINTS_PER_PACKET;
            }
            break;
    }
    
    if (expected_points > 0) {
        // 確保有足夠空間存儲點
        size_t current_size = points.size();
        points.resize(current_size + expected_points);
    }

    // 處理數據點
    int point_start = DATA_START_OFFSET + 4;  // 數據點開始位置
    int point_idx = points.size() - expected_points;
    
    for (int i = 0; i < POINTS_PER_PACKET; i++) {
        // 計算數據偏移量
        int data_offset = point_start + (i * 3);
        
        // 快速讀取強度和半徑
        uint8_t intensity = data[data_offset];

        // 跳過無效點    //JP-0428  intensity >255 或是 <=0的點就跳過不畫
        if ((intensity > 255) || (intensity <=0) ) {
             //ROS_INFO("Intensity out of range(0~ 255)! %d ", intensity);
            continue;
        }

        /*
        // William  0326
            eg. Integer:12 bits
                Decimal:4 bits
            data[data_offset+2] = 0x6A = 0110 1010
            data[data_offset+1] = 0x5D = 0101 1101
            integer part : 0110 1010 0101 = 1701
            decimal part : 1101 = 1 * 0.5 + 1 * 0.25 + 0 * 0.125 + 1 * 0.0625 = 0.8125 or 13/16 = 0.8125
            
        */
        float radius = (float)((data[data_offset+2] << 8) | data[data_offset+1])/32.0;

        if (radius <= 0) // Error check & bypass draw 0 value
        {
            // ROS_INFO("Radius less than 0! %f ", radius);
            continue;
        }

        if (REVISE){
            radius = (radius+START_READOUT)*0.15 - path_revise; //減去機構內光程距離 
        }else{
            radius = (radius+START_READOUT)*0.15;
        }

        // 計算仰角及其三角函數值
        if (REVISE){
            //修正SPDA對應角度  JP-0407
            if(elevation_start > 0){
                elevation = vu_table[i]/1000000;
            }else if (elevation_start < 0){
                elevation = vd_table[i]/1000000;
            }
        }else{
            elevation = elevation_start + (i * ELEVATION_STEP);
        }

        float rad_elevation = elevation * M_PI / 180.0f;
        float cos_elevation = cos(rad_elevation);
        float sin_elevation = sin(rad_elevation);
        
        // 計算笛卡爾坐標
        LidarPoint& point = points[point_idx++];
        point.valid = true;
        point.echo_num = echo_num;
        point.azimuth = azimuth;
        point.radius_in = radius;
        point.elevation = elevation;
        point.intensity = intensity;
        point.y = radius * cos_elevation * cos_azimuth; 
        point.x = radius * cos_elevation * sin_azimuth;  
        point.z = radius * sin_elevation;

        // 依據強度值計算顏色 -- JP0320
        // 將強度值歸一化到0-1範圍
        float normalized_intensity = point.intensity / 255.0f;
        normalized_intensity = std::min(std::max(normalized_intensity, 0.0f), 1.0f);
        
        // Adjust the hue mapping to compress Cyan-to-Green and Green-to-Yellow ranges
        float hue;
        if (normalized_intensity < 0.5f)
        {
            hue = normalized_intensity * 4.0f - 0.3f; // Blue to Cyan to Green (0.0 to 0.5, slower to Green)
        } // if
        else
        {
            hue = normalized_intensity * 4.0f + 0.3; // Green to Yellow to Red (0.5 to 1.0, faster to Red)
            hue = std::min(hue, 3.9f);             // Clamp to 3.9 so it stops at Red
        } // else

        int int_of_hue = floor(hue);
        float f = hue - int_of_hue;
        if (!(int_of_hue & 1))
            f = 1 - f; // if i is even // for smooth color transition

        float r, g, b;
        switch (int_of_hue % 4)
        { // Use modulo 4 to loop through the color transitions
        case 0:
            r = 0;
            g = 1 - f;
            b = 1;
            break; // Blue to Cyan
        case 1:
            r = 0;
            g = 1;
            b = 1 - f;
            break; // Cyan to Green
        case 2:
            r = 1 - f;
            g = 1;
            b = 0;
            break; // Green to Yellow
        case 3:
            r = 1;
            g = 1 - f;
            b = 0;
            break; // Yellow to Red
        default:
            r = 0;
            g = 0;
            b = 1;
            break; // Blue
        } // switch

        point.r = r;
        point.g = g;
        point.b = b;
        
        /*
        // 顏色映射：強度由高到低 -> 紅->橙->黃->綠->藍 // no smooth color transition
         if (normalized_intensity > 0.8f) {
             point.r = 1; point.g = 0; point.b = 0;  // 紅色（強度最高）
         } else if (normalized_intensity > 0.6f) {
             point.r = 1; point.g = 0.645; point.b = 0;  // 橙色
         } else if (normalized_intensity > 0.4f) {
             point.r = 1; point.g = 1; point.b = 0;  // 黃色
         } else if (normalized_intensity > 0.2f) {
             point.r = 0; point.g = 1; point.b = 0;  // 綠色
         } else {
             point.r = 0; point.g = 0; point.b = 1;  // 藍色（強度最低）
         } // else
       */
    }  // for loop

    // 如果實際點數少於預期，則調整大小 
    if (point_idx < points.size()) {
        points.resize(point_idx);
        //ROS_INFO("Resize point_idx: %d ", point_idx);
    }

}

// 創建PointCloud2消息，添加顏色映射
sensor_msgs::PointCloud2 create_point_cloud_msg(const std::vector<LidarPoint> &points)
{
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "stellar";
    msg.height = 1;
    msg.width = points.size();
    msg.is_bigendian = false;
    msg.is_dense = false;
    // 點的數據結構包含：x, y, z, intensity, echo_num, azimuth, rgb
    msg.point_step = 40; // 10 個 float: x, y, z, intensity, echo_num, azimuth, radius_in, r, g, b
    msg.row_step = msg.point_step * msg.width;

    // 僅創建一次字段定義
    static std::vector<sensor_msgs::PointField> fields;
    if (fields.empty())
    {
        sensor_msgs::PointField field;
        field.datatype = sensor_msgs::PointField::FLOAT32;
        field.count = 1;

        field.name = "x";
        field.offset = 0;
        fields.push_back(field);

        field.name = "y";
        field.offset = 4;
        fields.push_back(field);

        field.name = "z";
        field.offset = 8;
        fields.push_back(field);

        field.name = "intensity";
        field.offset = 12;
        fields.push_back(field);

        field.name = "echo_num";
        field.offset = 16;
        fields.push_back(field);

        field.name = "azimuth";
        field.offset = 20;
        fields.push_back(field);

        field.name = "radius_in";
        field.offset = 24;
        fields.push_back(field);

        // 添加RGB顏色字段
        // field.name = "rgb";
        // field.offset = 24;
        // field.datatype = sensor_msgs::PointField::UINT32;
        field.name = "r";
        field.offset = 28;
        fields.push_back(field);

        field.name = "g";
        field.offset = 32;
        fields.push_back(field);

        field.name = "b";
        field.offset = 36;
        fields.push_back(field);
    }

    msg.fields = fields;

    // 分配數據空間
    msg.data.resize(msg.row_step);
    // msg.data.resize(msg.row_step * msg.height);

    // 將強度值映射到顏色
    // 直接複製內存塊而不是單點賦值
    float *dst_ptr = reinterpret_cast<float *>(msg.data.data());
    for (const auto &point : points)
    {
        *dst_ptr++ = point.x;
        *dst_ptr++ = point.y;
        *dst_ptr++ = point.z;
        *dst_ptr++ = point.intensity;
        *dst_ptr++ = static_cast<float>(point.echo_num);
        *dst_ptr++ = static_cast<float>(point.azimuth);
        *dst_ptr++ = static_cast<float>(point.radius_in);
        *dst_ptr++ = static_cast<float>(point.r);
        *dst_ptr++ = static_cast<float>(point.g);
        *dst_ptr++ = static_cast<float>(point.b);
        // 將RGB打包成單一float
        // uint32_t rgb_packed = (static_cast<uint32_t>(point.r) << 16) |
        //                                                (static_cast<uint32_t>(point.g) << 8) |
        //                                               (static_cast<uint32_t>(point.b) );
        // *dst_ptr++ = *reinterpret_cast<float*>(&rgb_packed);
    }

    return msg;
}

// UDP接收線程函數
void udp_receiver_thread() {
    try {
        boost::asio::io_service io_service;
        udp::socket socket(io_service, udp::endpoint(udp::v4(), UDP_PORT));
        socket.set_option(boost::asio::socket_base::receive_buffer_size(8388608)); // 8MB 接收緩衝區
        
        // 預分配接收緩衝區和重用端點
        std::vector<uint8_t> recv_buffer(PACKET_SIZE);
        udp::endpoint remote_endpoint;
        
        // 設置socket為非阻塞模式
        socket.non_blocking(true);
        
        while (running) {
            boost::system::error_code error;
            size_t len = socket.receive_from(boost::asio::buffer(recv_buffer), remote_endpoint, 0, error);
            
            if (error == boost::asio::error::would_block) {
                // 沒有數據可讀，短暫休眠避免CPU佔用過高
                std::this_thread::sleep_for(std::chrono::microseconds(100));
                continue;
            } else if (error) {
                ROS_ERROR("Socket error: %s", error.message().c_str());
                continue;
            }
            
            if (len == PACKET_SIZE) {
                // 創建數據包並放入隊列
                UdpPacket packet;
                packet.data = recv_buffer;  // 這裡會複製數據
                packet.is_valid = true;
                
                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    packet_queue.push(std::move(packet));
                }
                
                packet_counter.fetch_add(1, std::memory_order_relaxed);
                queue_cv.notify_one();
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("UDP Receiver thread exception: %s", e.what());
        running = false;
    }
}

// 主處理函數
void udp_to_pointcloud2(ros::Publisher& pub, EchoMode echo_mode) {
    // 用於追踪掃描線狀態
    int line_count = 0;
    
    // 幀緩衝區 - 預先分配空間
    std::vector<LidarPoint> frame_points_buffer;
    frame_points_buffer.reserve(TOTAL_POINTS_PER_LINE * LINES_PER_FRAME);
    
    // 線掃描緩衝區
    std::vector<LidarPoint> line_points;
    line_points.reserve(TOTAL_POINTS_PER_LINE);
    static int frame_counter_per_sec = 0;
    static int frames_60 = 0;
    ros::Time last_report_time = ros::Time::now();
    static int parsed_points_counter = 0;

    static int csv_seq = 1;

    // 60 秒統計變數（累計點數與 frame 數）
    static ros::Time last_60s_report_time = ros::Time::now();
    static uint64_t upper_points_60 = 0;
    static uint64_t lower_points_60 = 0;
    static uint64_t total_points_60 = 0;

    #ifdef TEST
    // 性能監控變數

    // 起始測試時間，用於丟包率監控
    ros::Time test_start_time = ros::Time::now();

    // 丟包率監控變數（static 保留跨迴圈資料）
    static bool drop_monitor_started = false;
    static ros::Time drop_monitor_interval_start = ros::Time::now();
    static ros::Time drop_monitor_interval_end = ros::Time::now();
    static uint64_t drop_prev = 0;              // 上一區間的封包數（單位：60秒內接收的封包數）
    static uint64_t drop_interval_start_count = 0; // 當前區間起始時 packet_counter 的數值
	#endif
	
    while (ros::ok() && running) {
        UdpPacket packet;
        bool has_packet = false;
        
        // 從隊列中獲取數據包
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (!packet_queue.empty()) {
                packet = std::move(packet_queue.front());
                packet_queue.pop();
                has_packet = true;
            } else {
                // 等待新數據包，最多100ms
                queue_cv.wait_for(lock, std::chrono::milliseconds(100));
            }
        }
        
        if (!has_packet) {
            continue;
        }
        
        // 解析封包取得掃描線點，並判斷是否屬於上半部
        bool is_upper_packet = false;
        line_points.clear();  // 準備接收新的線掃描點
        
        parse_udp_packet(packet.data, line_points, is_upper_packet, echo_mode);
    
        if (line_points.empty()) {
            continue;  // 跳過沒有有效點的數據包
        }

        int frame_threshold = (echo_mode == ECHO_MODE_ALL)
        ? (LINES_PER_FRAME * 4)
        : (LINES_PER_FRAME * 2);
        
        #ifdef TEST	
        // 更新本區間內的點數統計
        parsed_points_counter += line_points.size();
        if (is_upper_packet)
            upper_points_60 += line_points.size();
        else
            lower_points_60 += line_points.size();
        total_points_60 += line_points.size();
        // 每 60 秒統計一次點數與 frame 數並寫入 CSV
        ros::Time now60 = ros::Time::now();
        if ((now60 - last_60s_report_time).toSec() >= 60.0) {
            char time_str[64];
            time_t rawtime = time(NULL);
            struct tm *timeinfo = localtime(&rawtime);
            strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);
            char date_str[16];
            strftime(date_str, sizeof(date_str), "%Y%m%d", timeinfo);

            double avg_points = 0.0;
            if (echo_mode == ECHO_MODE_ALL) {
                avg_points = static_cast<double>(total_points_60) / 60.0;
            } else if (echo_mode == ECHO_MODE_1ST || echo_mode == ECHO_MODE_2ND) {
                uint64_t selected_points = (echo_mode == ECHO_MODE_1ST) ? upper_points_60 : lower_points_60;
                avg_points = static_cast<double>(selected_points * 2) / 60.0;
            }
            double avg_frame = static_cast<double>(frames_60) / 60.0;

            printf("[Current time] %s, Average Total Point: %.2f, Average Frame: %.2f\n", 
                   time_str, avg_points, avg_frame);
            ROS_INFO_STREAM("[Current time] " << time_str 
                << ", Average Total Point " << std::fixed << std::setprecision(2) << avg_points 
                << ", Average Frame " << std::fixed << std::setprecision(2) << avg_frame);

            char csv_filename[128];
            snprintf(csv_filename, sizeof(csv_filename), "freq_log_%s_%d.csv", date_str, csv_seq);
            std::ofstream log_file(csv_filename, std::ios::app);
            if (log_file.is_open()) {
                log_file << time_str << "," << avg_points << "," << avg_frame << "\n";
                log_file.close();
            }
            csv_seq++;

            // 重置 60 秒統計變數
            upper_points_60 = 0;
            lower_points_60 = 0;
            total_points_60 = 0;
            frames_60 = 0;
            last_60s_report_time = now60;

        }

        // 丟包率監控邏輯：
        // 從程式啟動 10 秒後開始，每 60 秒計算一次丟包率，計算公式：
        // drop_rate = (|本區間收到的封包數 - 上一區間收到的封包數|) / (本區間數 + 上一區間數) * 100%
        double elapsed_since_start = (ros::Time::now() - test_start_time).toSec();
        if (elapsed_since_start >= 10.0 && !drop_monitor_started) {
            drop_monitor_started = true;
            drop_monitor_interval_start = ros::Time::now();
            drop_monitor_interval_end = drop_monitor_interval_start + ros::Duration(60.0);
            drop_interval_start_count = packet_counter.load();
        }
        if (drop_monitor_started && ros::Time::now() >= drop_monitor_interval_end) {
            // 取得本區間收到的封包數（僅限此 60 秒區間）
            uint64_t current_interval_count = packet_counter.load() - drop_interval_start_count;
            uint64_t combined = current_interval_count + drop_prev;
            double drop_rate = 0.0;
            if (drop_prev == 0 || combined == 0) {
                drop_rate = 0.0;
            } else {
                // 若二者差值為負，取絕對值再計算 drop rate
                uint64_t diff = static_cast<uint64_t>(std::abs((long long)current_interval_count - (long long)drop_prev));
                drop_rate = (double)diff / (double)combined * 100.0;
            }
            // 輸出上一區間、這區間及合計數值與計算出的 drop rate
            printf("First interval: %ju, Second interval: %ju, Combined: %ju, Drop rate: %.8f%%\n",
                   drop_prev, current_interval_count, combined, drop_rate);
            {
                std::ofstream drop_log("drop_rate_log.csv", std::ios::app);
                if (drop_log.is_open()) {
                    time_t now_t = time(NULL);
                    char time_buf[64];
                    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", localtime(&now_t));
                    drop_log << time_buf << "," << drop_prev << "," << current_interval_count << ","
                             << combined << "," << std::fixed << std::setprecision(8) << drop_rate << "\n";
                    drop_log.close();
                }
            }
            // 將本區間的封包數存入 drop_prev，下次比較使用
            drop_prev = current_interval_count;
            // 重置下一區間的起始時間與封包起始值
            drop_monitor_interval_start = ros::Time::now();
            drop_monitor_interval_end = drop_monitor_interval_start + ros::Duration(60.0);
            drop_interval_start_count = packet_counter.load();
        }
#endif

    bool should_publish = false;  

    #ifdef REFRESH_BY_PSN
        // 原始條件：收滿行數
        if (line_count >= frame_threshold) {
            ROS_DEBUG(" threshold triggers frame at line_count %d", line_count);
            should_publish = true;
        }
        // 新增條件：wrap-around
        else if (pkt_psn < last_pkt_psn) {
            ROS_DEBUG(" wrap-around triggers frame at psn %u→%u", last_pkt_psn, pkt_psn);
            should_publish = true;
        }
    #else
        // 非 REFRESH_BY_PSN：只用原始行數閾值
        if (line_count >= frame_threshold) {
            ROS_DEBUG("threshold triggers frame at line_count %d", line_count);
            should_publish = true;
        }
    #endif

        //  發布並重置 
        if (should_publish) {
            ROS_DEBUG("Publishing frame with %d lines (%zu points)", line_count, frame_points_buffer.size());
            sensor_msgs::PointCloud2 pcl_msg = create_point_cloud_msg(frame_points_buffer);
            pub.publish(pcl_msg);
            frame_counter.fetch_add(1, std::memory_order_relaxed);
			frame_counter_per_sec++;
			frames_60++;

			// 5 秒效能輸出
            ros::Time current_time = ros::Time::now();
            if ((current_time - last_report_time).toSec() >= 5.0) {
                double elapsed = (current_time - performance_start_time).toSec();
                double fps = frame_counter.load() / elapsed;
                int packets = packet_counter.load();
                ROS_INFO("Performance: %.2f fps, processed %d packets, queue size: %zu", fps, packets, packet_queue.size());
                last_report_time = current_time;
            } 
            // 清空緩衝區，重置計數器
            frame_points_buffer.clear();
            line_count = 0;
        }
    // 將掃描線點添加到幀緩衝區
    frame_points_buffer.insert(frame_points_buffer.end(), line_points.begin(), line_points.end());
    line_count++;
    //  更新 PSN 基準 
    last_pkt_psn = pkt_psn;
    // 允許ROS處理回調
    ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    // 檢查是否需要顯示幫助訊息
    if (argc == 2 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        printf("使用方法: roslaunch udp_receiver start.launch [echo_mode:=值]\n\n");
        printf("參數說明:\n");
        printf(" echo_mode:=0 顯示所有回波 (同時顯示第1回波與第2回波，預設)\n");
        printf(" echo_mode:=1 僅顯示第1回波\n");
        printf(" echo_mode:=2 僅顯示第2回波\n\n");
        printf("範例:\n");
        printf(" roslaunch udp_receiver start.launch # 同時顯示第1回波與第2回波\n");
        printf(" roslaunch udp_receiver start.launch echo_mode:=1 # 只顯示第1回波\n");
        printf(" roslaunch udp_receiver start.launch echo_mode:=2 # 只顯示第2回波\n");
        return 0;
    }
    
    ros::init(argc, argv, "udp_to_pointcloud2");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 從參數獲取配置
    int echo_mode_param = 0;  // 默認顯示所有回波
    private_nh.param("echo_mode", echo_mode_param, 0);
    private_nh.param("debug", DEBUG, false);
    
    EchoMode echo_mode = static_cast<EchoMode>(echo_mode_param);
    
    // 創建發布者
    std::string topic_name;
    switch(echo_mode) {
        case ECHO_MODE_1ST:
            topic_name = "/pointcloud_udp_1st_echo";
            break;
        case ECHO_MODE_2ND:
            topic_name = "/pointcloud_udp_2nd_echo";
            break;
        case ECHO_MODE_ALL:
        default:
            topic_name = "/pointcloud_udp";
            break;
    }
    
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 10);
    
    // 顯示配置信息
    std::string echo_mode_str;
    switch(echo_mode) {
        case ECHO_MODE_1ST:
            echo_mode_str = "First Echo Only (ECHO_1ST)";
            break;
        case ECHO_MODE_2ND:
            echo_mode_str = "Second Echo Only (ECHO_2ND)";
            break;
        case ECHO_MODE_ALL:
        default:
            echo_mode_str = "All Echoes";
            break;
    }
    
    ROS_INFO("Starting LiDAR UDP to PointCloud2 node (Optimized)...");
    ROS_INFO("Echo Mode: %s", echo_mode_str.c_str());
    ROS_INFO("Configuration: Collecting %d lines per frame", LINES_PER_FRAME);
    ROS_INFO("Debug mode: %s", DEBUG ? "ON" : "OFF");
    
    try {
        // 初始化性能監控
        performance_start_time = ros::Time::now();
        
        // 啟動接收線程
        std::thread receiver(udp_receiver_thread);
        
        // 啟動 DIFO packet receiver 線程
        //std::thread difo_thread(difo_udp_receiver_thread);

        // 主線程處理數據
        udp_to_pointcloud2(pub, echo_mode);
        
        // 清理
        running = false;
        if (receiver.joinable()) {
            receiver.join();
        }
        //if (difo_thread.joinable()) {
        //    difo_thread.join();
        //}
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}

