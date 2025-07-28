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
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

namespace show_radius
{

// 構造函數
PointRadiusTool::PointRadiusTool()
  : Tool()
  , info_label_(nullptr)
  , rubber_band_(nullptr)
  , is_selecting_(false)
{
  shortcut_key_ = 'r';  // 快捷鍵：r
  
  // 初始化區域統計
  region_stats_.valid = false;
  region_stats_.total_points = 0;
  
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
  
  if (rubber_band_)
  {
    delete rubber_band_;
    rubber_band_ = nullptr;
  }
  
  // 取消訂閱點雲
  if (pointcloud_sub_)
  {
    pointcloud_sub_.shutdown();
  }
}

// 正確覆蓋 Tool 類的 onInitialize 方法
void PointRadiusTool::onInitialize()
{
  ROS_INFO("PointRadiusTool::onInitialize() called");
  createLabel();
  createRubberBand();
  
  // 訂閱點雲話題
  ros::NodeHandle nh;
  pointcloud_sub_ = nh.subscribe("/pointcloud_udp", 1, &PointRadiusTool::pointCloudCallback, this);
}

// 點雲回調
void PointRadiusTool::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  latest_cloud_ = cloud;
}

// 創建橡皮筋選擇框
void PointRadiusTool::createRubberBand()
{
  if (!context_->getViewManager()->getRenderPanel())
  {
    ROS_ERROR("RenderPanel is null!");
    return;
  }
  
  rubber_band_ = new QRubberBand(QRubberBand::Rectangle, context_->getViewManager()->getRenderPanel());
  QPalette palette;
  palette.setBrush(QPalette::Highlight, QBrush(QColor(0, 160, 230, 120)));
  rubber_band_->setPalette(palette);
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
  
  info_label_->setText("請拖動選擇區域");
  info_label_->adjustSize();
  info_label_->setVisible(false);  // 初始時不可見
  
  ROS_INFO("Label created successfully");
}

// 激活工具
void PointRadiusTool::activate()
{
  ROS_INFO("PointRadiusTool activated");
  setStatus("拖動選擇區域以顯示點雲統計");
  region_stats_.valid = false;
  is_selecting_ = false;
  
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
  
  // 確保橡皮筋已創建
  if (!rubber_band_)
  {
    createRubberBand();
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
  
  if (rubber_band_)
  {
    rubber_band_->hide();
  }
  
  is_selecting_ = false;
}

// 獲取點擊位置的點信息
bool PointRadiusTool::getPointInfo(rviz::ViewportMouseEvent& event, int x, int y, Ogre::Vector3& position)
{
  try {
    ROS_INFO("Getting point info for position (%d, %d)", x, y);
    
    rviz::SelectionManager* selection_manager = context_->getSelectionManager();
    if (!selection_manager) {
      ROS_WARN("Selection manager is null");
      return false;
    }
    
    // 使用 3D 游標位置代替直接選擇點
    bool success = selection_manager->get3DPoint(event.viewport, x, y, position);
    
    if (!success) {
      ROS_WARN("Failed to get 3D point");
      return false;
    }
    
    ROS_INFO("Got 3D point: (%f, %f, %f)", position.x, position.y, position.z);
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in getPointInfo: %s", e.what());
    return false;
  } catch (...) {
    ROS_ERROR("Unknown exception in getPointInfo");
    return false;
  }
}

// 計算選擇區域內的點雲統計
void PointRadiusTool::calculateRegionStats(const QPoint& start, const QPoint& end)
{
  if (!latest_cloud_ || latest_cloud_->data.empty())
  {
    ROS_WARN("No point cloud data available");
    region_stats_.valid = false;
    return;
  }
  
  // 轉換為PCL點雲 (使用PointXYZ類型，只關注坐標)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*latest_cloud_, *cloud);
  
  if (cloud->empty())
  {
    ROS_WARN("Converted point cloud is empty");
    region_stats_.valid = false;
    return;
  }
  
  // 創建一個3D框以過濾點
  pcl::CropBox<pcl::PointXYZ> crop_box;
  crop_box.setInputCloud(cloud);
  
  // 設置過濾框的最小點和最大點
  // 我們需要確保選擇點的順序正確（最小x/y/z到最大x/y/z）
  Eigen::Vector4f min_point, max_point;
  
  min_point[0] = std::min(select_start_3d_.x, select_end_3d_.x);
  min_point[1] = std::min(select_start_3d_.y, select_end_3d_.y);
  min_point[2] = std::min(select_start_3d_.z, select_end_3d_.z);
  min_point[3] = 1.0f;
  
  max_point[0] = std::max(select_start_3d_.x, select_end_3d_.x);
  max_point[1] = std::max(select_start_3d_.y, select_end_3d_.y);
  max_point[2] = std::max(select_start_3d_.z, select_end_3d_.z);
  max_point[3] = 1.0f;
  
  // 增加一些緩衝區以確保選擇區域包含所有相關點
  const float buffer = 0.1f;  // 10cm緩衝區
  min_point[0] -= buffer;
  min_point[1] -= buffer;
  min_point[2] -= buffer;
  
  max_point[0] += buffer;
  max_point[1] += buffer;
  max_point[2] += buffer;
  
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);
  
  // 提取選擇區域內的點
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  crop_box.filter(*cropped_cloud);
  
  // 計算統計
  region_stats_.total_points = cropped_cloud->size();
  region_stats_.valid = true;
  
  ROS_INFO("Region stats: total points = %d", region_stats_.total_points);
}

// 處理鼠標事件
int PointRadiusTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  try {
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
    
    // 檢查橡皮筋是否存在，如果不存在則創建
    if (!rubber_band_)
    {
      ROS_WARN("Rubber band is null, creating it now");
      createRubberBand();
      if (!rubber_band_)
      {
        ROS_ERROR("Failed to create rubber band!");
        return Render;
      }
    }
    
    // 處理鼠標左鍵按下（開始選擇）
    if (event.leftDown())
    {
      ROS_INFO("Left mouse button pressed at (%d, %d)", event.x, event.y);
      
      // 清除上一次的結果顯示
      if (info_label_->isVisible())
      {
        info_label_->setVisible(false);
      }
      
      // 獲取起點的3D坐標
      Ogre::Vector3 position;
      if (getPointInfo(event, event.x, event.y, position))
      {
        select_start_ = QPoint(event.x, event.y);
        select_start_3d_ = position;
        
        // 顯示起點信息
        QString info = QString("起點: (%1, %2, %3)")
                         .arg(position.x, 0, 'f', 2)
                         .arg(position.y, 0, 'f', 2)
                         .arg(position.z, 0, 'f', 2);
        
        info_label_->setText(info);
        info_label_->adjustSize();
        info_label_->move(event.x + 10, event.y - 30);
        info_label_->raise();
        info_label_->setVisible(true);
        
        // 開始選擇模式
        is_selecting_ = true;
        rubber_band_->setGeometry(QRect(select_start_, QSize()));
        rubber_band_->show();
      }
    }
    // 處理鼠標拖動（更新選擇區域）
    else if (event.left() && is_selecting_)
    {
      select_end_ = QPoint(event.x, event.y);
      
      // 更新橡皮筋大小
      rubber_band_->setGeometry(QRect(select_start_, select_end_).normalized());
      
      // 獲取終點的3D坐標
      Ogre::Vector3 position;
      if (getPointInfo(event, event.x, event.y, position))
      {
        select_end_3d_ = position;
        
        // 更新顯示信息
        QString info = QString("起點: (%1, %2, %3)\n終點: (%4, %5, %6)")
                         .arg(select_start_3d_.x, 0, 'f', 2)
                         .arg(select_start_3d_.y, 0, 'f', 2)
                         .arg(select_start_3d_.z, 0, 'f', 2)
                         .arg(position.x, 0, 'f', 2)
                         .arg(position.y, 0, 'f', 2)
                         .arg(position.z, 0, 'f', 2);
        
        info_label_->setText(info);
        info_label_->adjustSize();
        info_label_->move(event.x + 10, event.y - 60);
      }
    }
    // 處理鼠標左鍵釋放（完成選擇）
    else if (event.leftUp() && is_selecting_)
    {
      ROS_INFO("Left mouse button released at (%d, %d)", event.x, event.y);
      is_selecting_ = false;
      
      // 隱藏橡皮筋
      rubber_band_->hide();
      
      // 計算選擇區域內的點雲統計
      calculateRegionStats(select_start_, select_end_);
      
      if (region_stats_.valid)
      {
        // 顯示統計結果
        QString stats = QString("區域統計:\n總點數: %1")
                          .arg(region_stats_.total_points);
        
        info_label_->setText(stats);
        info_label_->adjustSize();
        
        // 計算標籤位置（盡量不擋住所選區域）
        QRect selection_rect = QRect(select_start_, select_end_).normalized();
        int label_x, label_y;
        
        // 嘗試將標籤放在選擇區域的右側
        label_x = selection_rect.right() + 10;
        label_y = selection_rect.center().y() - info_label_->height() / 2;
        
        // 如果標籤會超出窗口右邊界，則放在左側
        QSize window_size = context_->getViewManager()->getRenderPanel()->size();
        if (label_x + info_label_->width() > window_size.width())
        {
          label_x = selection_rect.left() - info_label_->width() - 10;
        }
        
        // 如果標籤會超出窗口上下邊界，調整垂直位置
        if (label_y < 0)
        {
          label_y = 0;
        }
        else if (label_y + info_label_->height() > window_size.height())
        {
          label_y = window_size.height() - info_label_->height();
        }
        
        info_label_->move(label_x, label_y);
        info_label_->raise();
        info_label_->setVisible(true);
      }
      else
      {
        info_label_->setText("無法計算區域統計");
        info_label_->adjustSize();
        info_label_->move(event.x + 10, event.y - 30);
        info_label_->setVisible(true);
      }
    }
    
    return Render;
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in processMouseEvent: %s", e.what());
    return Render;
  } catch (...) {
    ROS_ERROR("Unknown exception in processMouseEvent");
    return Render;
  }
}

} // end namespace show_radius

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(show_radius::PointRadiusTool, rviz::Tool)