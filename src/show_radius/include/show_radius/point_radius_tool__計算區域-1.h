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
#include <QRubberBand>
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

  // 覆蓋 Tool 類的虛擬方法
  void onInitialize() override;

  // 當鼠標事件發生時會被調用的方法
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

  // 工具啟動/停用時的處理
  void activate() override;
  void deactivate() override;

protected:
  // 獲取點擊位置的點信息
  bool getPointInfo(rviz::ViewportMouseEvent& event, int x, int y, Ogre::Vector3& position);
  
  // 計算選擇區域內的點雲統計
  void calculateRegionStats(const QPoint& start, const QPoint& end);
  
  // 創建標籤方法
  void createLabel();
  
  // 創建橡皮筋選擇框
  void createRubberBand();

  // 用於顯示提示的標籤
  QLabel* info_label_;
  
  // 橡皮筋選擇框
  QRubberBand* rubber_band_;
  
  // 是否正在拖動選擇
  bool is_selecting_;
  
  // 選擇起點和終點
  QPoint select_start_;
  QPoint select_end_;
  
  // 3D空間中的選擇起點和終點
  Ogre::Vector3 select_start_3d_;
  Ogre::Vector3 select_end_3d_;
  
  // 區域統計結果
  struct RegionStats {
    int total_points;    // 區域內總點數
    bool valid;
  } region_stats_;
  
  // 點雲訂閱者
  ros::Subscriber pointcloud_sub_;
  
  // 最新的點雲消息
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  
  // 點雲回調
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
};

} // namespace show_radius

#endif // POINT_RADIUS_TOOL_H