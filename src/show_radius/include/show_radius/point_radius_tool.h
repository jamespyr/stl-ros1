#ifndef POINT_RADIUS_TOOL_H
#define POINT_RADIUS_TOOL_H

#include <ros/ros.h>
#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/load_resource.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <sensor_msgs/PointCloud2.h>
#include <QMessageBox>
#include <QLabel>
#include <QPainter>
#include <QTimer>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>

namespace show_radius
{

class PointRadiusTool : public rviz::Tool
{
public:
  PointRadiusTool();
  ~PointRadiusTool() override;

  // 確保 onInitialize 是公共的，覆蓋 Tool 類的虛擬方法
  void onInitialize() override;

  // 當鼠標事件發生時會被調用的方法
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

  // 工具啟動/停用時的處理
  void activate() override;
  void deactivate() override;

protected:
  // 獲取和顯示點雲信息的方法
  bool getPointInfo(rviz::ViewportMouseEvent& event, int x, int y);
  
  // 創建標籤方法
  void createLabel();
  
  // 初始化点云订阅
  void initPointCloudSubscribers();
  
  // 检查话题是否有数据
  void checkActiveTopics();
  
  // 用於顯示提示的標籤
  QLabel* info_label_;
  
  // 上次點擊的點信息
  struct PointInfo {
    float x, y, z;
    float radius;
    float intensity; // 新增：intensity值
    bool valid;
  } last_point_;
  
  // 控制點擊響應間隔
  ros::Time last_click_time_;
  
  // 上次顯示的標籤位置
  int last_label_x_;
  int last_label_y_;

  // 標籤顯示持續時間（秒）
  double label_display_duration_;
  
  // 點雲訂閱者和最新點雲數據 - 新增
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  
  // 记录是否可以从点云获取intensity - 新增
  bool has_intensity_;
  
  // 点云话题列表和多个订阅者
  std::vector<std::string> pointcloud_topics_;
  std::vector<ros::Subscriber> pointcloud_subs_;
  
  // 当前活跃的点云话题索引
  int active_topic_index_;
};

} // namespace show_radius

#endif // POINT_RADIUS_TOOL_H