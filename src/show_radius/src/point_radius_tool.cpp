#include "show_radius/point_radius_tool.h"
#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/selection/forwards.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <QApplication>
#include <QTimer>
#include <QColor>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

namespace show_radius
{

// 構造函數
PointRadiusTool::PointRadiusTool()
  : Tool()
  , info_label_(nullptr)
  , last_click_time_(ros::Time(0))
  , label_display_duration_(3.0)  // 顯示3秒
  , has_intensity_(false)
  , active_topic_index_(-1)  // 初始化为-1，表示没有活跃话题
{
  shortcut_key_ = 'r';  // 快捷鍵：r
  
  // 初始化上次點擊的點
  last_point_.valid = false;
  last_point_.x = 0.0f;
  last_point_.y = 0.0f;
  last_point_.z = 0.0f;
  last_point_.radius = 0.0f;
  last_point_.intensity = 0.0f;
  
  // 初始化点云话题列表
  pointcloud_topics_.push_back("/pointcloud_udp");      // ECHO_MODE_ALL
  pointcloud_topics_.push_back("/pointcloud_udp_1st_echo");  // ECHO_MODE_1ST
  pointcloud_topics_.push_back("/pointcloud_udp_2nd_echo");  // ECHO_MODE_2ND
  
  ROS_INFO("PointRadiusTool constructed");
}

// 析構函數
PointRadiusTool::~PointRadiusTool()
{
  if (info_label_)
  {
    delete info_label_;
    info_label_ = nullptr;
  }
  
  // 取消所有点云订阅
  for (auto& sub : pointcloud_subs_)
  {
    if (sub) sub.shutdown();
  }
  pointcloud_subs_.clear();
}

// 初始化点云订阅者
void PointRadiusTool::initPointCloudSubscribers()
{
  ros::NodeHandle nh;
  pointcloud_subs_.clear();
  
  for (const auto& topic : pointcloud_topics_)
  {
    // 为每个可能的话题创建一个订阅者
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      topic, 1, 
      [this, topic](const sensor_msgs::PointCloud2::ConstPtr& cloud) {
        // 检查这个消息来自哪个话题
        auto it = std::find(pointcloud_topics_.begin(), pointcloud_topics_.end(), topic);
        if (it != pointcloud_topics_.end())
        {
          int index = std::distance(pointcloud_topics_.begin(), it);
          
          // 更新活跃话题索引和最新点云
          if (index != active_topic_index_)
          {
            ROS_INFO("Switching to active topic: %s", topic.c_str());
            active_topic_index_ = index;
          }
          
          latest_cloud_ = cloud;
          
          // 检查点云是否包含intensity字段
          bool found_intensity = false;
          for (const auto& field : cloud->fields)
          {
            if (field.name == "intensity")
            {
              found_intensity = true;
              break;
            }
          }
          
          has_intensity_ = found_intensity;
          if (!has_intensity_)
          {
            ROS_WARN_ONCE("The point cloud does not have intensity field");
          }
        }
      }
    );
    
    pointcloud_subs_.push_back(sub);
    ROS_INFO("Subscribed to potential topic: %s", topic.c_str());
  }
}

// 检查活跃话题
void PointRadiusTool::checkActiveTopics()
{
  // 这个函数用于在没有活跃话题时输出提示
  if (active_topic_index_ < 0)
  {
    ROS_WARN_THROTTLE(5, "No active point cloud topics detected. Make sure udp_receiver is running.");
  }
  else
  {
    ROS_INFO_THROTTLE(10, "Active point cloud topic: %s", 
                     pointcloud_topics_[active_topic_index_].c_str());
  }
}

// 正確覆蓋 Tool 類的 onInitialize 方法
void PointRadiusTool::onInitialize()
{
  ROS_INFO("PointRadiusTool::onInitialize() called");
  createLabel();
  
  // 初始化点云订阅
  initPointCloudSubscribers();
  
  // 创建定时器，定期检查活跃话题
  // 注意：这里使用QtTimer而不是ros::Timer，因为RViz插件通常在Qt环境中运行
  QTimer* check_timer = new QTimer(this);
  connect(check_timer, &QTimer::timeout, [this]() {
    this->checkActiveTopics();
  });
  check_timer->start(5000); // 每5秒检查一次
}

// 創建標籤
void PointRadiusTool::createLabel()
{
  ROS_INFO("Creating info label...");
  if (!context_)
  {
    ROS_ERROR("Context is null!");
    return;
  }
  
  if (!context_->getViewManager())
  {
    ROS_ERROR("ViewManager is null!");
    return;
  }
  
  if (!context_->getViewManager()->getRenderPanel())
  {
    ROS_ERROR("RenderPanel is null!");
    return;
  }

  // 創建標籤
  info_label_ = new QLabel(context_->getViewManager()->getRenderPanel());
  info_label_->setFrameStyle(QFrame::Box | QFrame::Raised);
  info_label_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
  info_label_->setAutoFillBackground(true);
  
  QPalette palette = info_label_->palette();
  palette.setColor(QPalette::Window, QColor(30, 30, 30, 200));  // 半透明黑色背景
  palette.setColor(QPalette::WindowText, Qt::white);  // 白色文字
  info_label_->setPalette(palette);
  
  info_label_->setText("距離: --.- m");
  info_label_->adjustSize();
  info_label_->setVisible(false);  // 初始時不可見
  
  ROS_INFO("Label created successfully");
}

// 激活工具
void PointRadiusTool::activate()
{
  ROS_INFO("PointRadiusTool activated");
  setStatus("點擊點雲上的點來顯示其距離值和強度值");
  last_point_.valid = false;
  
  // 確保標籤已創建
  if (!info_label_)
  {
    createLabel();
  }
  
  if (info_label_)
  {
    info_label_->setVisible(false);
  }
  else
  {
    ROS_WARN("Info label is still null after activation");
  }
}

// 停用工具
void PointRadiusTool::deactivate()
{
  ROS_INFO("PointRadiusTool deactivated");
  if (info_label_)
  {
    info_label_->setVisible(false);
  }
}

// 處理鼠標事件
int PointRadiusTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  // 只響應左鍵點擊
  if (event.leftDown())
  {
    ROS_INFO("Left mouse button clicked at (%d, %d)", event.x, event.y);
    ros::Time now = ros::Time::now();
    
    // 檢查是否過於頻繁點擊（防抖動）
    if ((now - last_click_time_).toSec() < 0.2)
    {
      return Render;
    }
    
    last_click_time_ = now;
    
    // 檢查標籤是否存在，如果不存在則創建
    if (!info_label_)
    {
      ROS_WARN("Info label is null, creating it now");
      createLabel();
      if (!info_label_)
      {
        ROS_ERROR("Failed to create info label!");
        return Render;
      }
    }
    
    // 獲取點擊位置的點信息
    if (getPointInfo(event, event.x, event.y))
    {
      ROS_INFO("Got point info: radius = %f, position = (%f, %f, %f), intensity = %f", 
               last_point_.radius, last_point_.x, last_point_.y, last_point_.z, last_point_.intensity);
      last_label_x_ = event.x;
      last_label_y_ = event.y;
      
      // 创建显示信息 - 包括半径, (x,y,z) 坐标和强度值 (已移除话题信息)
      QString info;
      
      // 记录话题信息到日志，但不显示在标签上
      if (active_topic_index_ >= 0 && active_topic_index_ < pointcloud_topics_.size())
      {
        ROS_INFO("Active topic: %s", pointcloud_topics_[active_topic_index_].c_str());
      }
      else
      {
        ROS_WARN("No active point cloud topic detected");
      }

      if (has_intensity_)
      {
        info = QString("距離: %1 m\n座標: (%2, %3, %4)\n強度: %5")
                 .arg(last_point_.radius, 0, 'f', 2)
                 .arg(last_point_.x, 0, 'f', 2)
                 .arg(last_point_.y, 0, 'f', 2)
                 .arg(last_point_.z, 0, 'f', 2)
                 .arg(last_point_.intensity, 0, 'f', 2);
      }
      else
      {
        info = QString("距離: %1 m\n座標: (%2, %3, %4)\n強度: 不可用")
                 .arg(last_point_.radius, 0, 'f', 2)
                 .arg(last_point_.x, 0, 'f', 2)
                 .arg(last_point_.y, 0, 'f', 2)
                 .arg(last_point_.z, 0, 'f', 2);
      }
      
      // 設置標籤位置和內容
      info_label_->setText(info);
      info_label_->adjustSize();
      
      // 確保標籤不超出視窗邊界
      int label_x = last_label_x_ + 10;  // 偏移量，避免遮擋鼠標
      int label_y = last_label_y_ - 30;  // 向上偏移，更容易看到
      
      // 邊界檢查
      QSize label_size = info_label_->size();
      QSize window_size = context_->getViewManager()->getRenderPanel()->size();
      
      if (label_x + label_size.width() > window_size.width())
        label_x = window_size.width() - label_size.width();
      
      if (label_y < 0)
        label_y = 0;
      else if (label_y + label_size.height() > window_size.height())
        label_y = window_size.height() - label_size.height();
      
      info_label_->move(label_x, label_y);
      info_label_->raise();  // 確保標籤顯示在最上層
      info_label_->setVisible(true);
      
      ROS_INFO("Label displayed at (%d, %d): %s", label_x, label_y, info.toStdString().c_str());
      
      // 設置定時器，3秒後隱藏標籤
      QTimer::singleShot(label_display_duration_ * 1000, [this]() {
        if (info_label_)
        {
          info_label_->setVisible(false);
          ROS_INFO("Label hidden after timeout");
        }
      });
    }
    else
    {
      ROS_WARN("Failed to get point info");
    }
  }
  
  return Render;
}

// 獲取點擊位置的點信息
bool PointRadiusTool::getPointInfo(rviz::ViewportMouseEvent& event, int x, int y)
{
  try {
    ROS_INFO("Getting point info for position (%d, %d)", x, y);
    
    rviz::SelectionManager* selection_manager = context_->getSelectionManager();
    if (!selection_manager) {
      ROS_WARN("Selection manager is null");
      return false;
    }
    
    // 使用 3D 游標位置代替直接選擇點
    Ogre::Vector3 position;
    bool success = selection_manager->get3DPoint(event.viewport, x, y, position);
    
    if (!success) {
      ROS_WARN("Failed to get 3D point");
      return false;
    }
    
    ROS_INFO("Got 3D point: (%f, %f, %f)", position.x, position.y, position.z);
    
    // 計算距離作為半徑值
    float radius = position.length();
    
    // 保存點信息
    last_point_.x = position.x;
    last_point_.y = position.y;
    last_point_.z = position.z;
    last_point_.radius = radius;
    last_point_.valid = true;
    
    // 嘗試從點雲數據中獲取強度值
    last_point_.intensity = 0.0f; // 默認值
    
    if (latest_cloud_ && !latest_cloud_->data.empty() && has_intensity_)
    {
      // 將ROS點雲轉換為PCL點雲
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*latest_cloud_, *cloud);
      
      if (!cloud->empty())
      {
        // 查找最接近點擊位置的點
        float min_dist = std::numeric_limits<float>::max();
        int closest_idx = -1;
        
        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
          const auto& pt = cloud->points[i];
          float dist = std::sqrt(
            std::pow(pt.x - position.x, 2) +
            std::pow(pt.y - position.y, 2) +
            std::pow(pt.z - position.z, 2)
          );
          
          if (dist < min_dist)
          {
            min_dist = dist;
            closest_idx = i;
          }
        }
        
        // 如果找到了最近的點，獲取其強度值
        if (closest_idx >= 0 && closest_idx < cloud->points.size())
        {
          last_point_.intensity = cloud->points[closest_idx].intensity;
          ROS_INFO("Found intensity value: %f", last_point_.intensity);
        }
        else
        {
          ROS_WARN("Could not find closest point in cloud");
        }
      }
      else
      {
        ROS_WARN("Converted point cloud is empty");
      }
    }
    else
    {
      if (!latest_cloud_)
        ROS_WARN_ONCE("No point cloud data available");
      else if (!has_intensity_)
        ROS_WARN_ONCE("Point cloud does not have intensity field");
    }
    
    ROS_INFO("Calculated radius: %f", radius);
    
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in getPointInfo: %s", e.what());
    return false;
  } catch (...) {
    ROS_ERROR("Unknown exception in getPointInfo");
    return false;
  }
}

} // end namespace show_radius

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(show_radius::PointRadiusTool, rviz::Tool)