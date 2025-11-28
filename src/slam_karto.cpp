/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "localize_karto/Grid.h"
#include "localize_karto/Pose.h"
#include "localize_karto/correlation_scan_match.h"

typedef enum
{
  GridStates_Unknown = 0,
  GridStates_Occupied = 100,
  GridStates_Free = 0
} GridStates;

class SlamKarto : public rclcpp::Node
{
public:
  SlamKarto();
  ~SlamKarto() override;

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg);
  bool getLaserPose(karto::Pose2 & karto_pose, const rclcpp::Time & stamp, const std::string & frame_id);
  karto::LaserRangeFinder * getLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan);
  bool addScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan, karto::Pose2 & karto_pose);

  void mapReceived(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg);
  void convertMap(const nav_msgs::msg::OccupancyGrid & map_msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  karto::Pose2 manual_pose_;
  bool has_manual_pose_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr matched_pose_pub_;

  bool first_map_only_;
  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  int throttle_scans_;
  double rangeThreshold_;

  karto::Mapper * mapper_;
  karto::ScanMatcher * scanmatcher_;
  karto::CorrelationGrid * m_pCorrelationGrid_;

  std::map<std::string, karto::LaserRangeFinder *> lasers_;
  bool first_map_received_;
  int laser_count_;
};

SlamKarto::SlamKarto()
: Node("slam_karto"),
  first_map_only_(true),
  throttle_scans_(100),
  rangeThreshold_(15.0),
  mapper_(nullptr),
  scanmatcher_(nullptr),
  m_pCorrelationGrid_(nullptr),
  first_map_received_(false),
  laser_count_(0),
  has_manual_pose_(false)
{
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  throttle_scans_ = this->declare_parameter<int>("throttle_scans", 100);
  first_map_only_ = this->declare_parameter<bool>("first_map_only", true);
  first_map_received_ = this->declare_parameter<bool>("first_map_received", false);
  rangeThreshold_ = this->declare_parameter<double>("rangeThreshold", 15.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Frames configured - map: %s, odom: %s, base: %s",
    map_frame_.c_str(),
    odom_frame_.c_str(),
    base_frame_.c_str());

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose",
    10,
    std::bind(&SlamKarto::initialPoseCallback, this, std::placeholders::_1));

  laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan",
    rclcpp::SensorDataQoS(),
    std::bind(&SlamKarto::laserCallback, this, std::placeholders::_1));

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_qos.reliable();
  map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "test/map",
    map_qos,
    std::bind(&SlamKarto::mapReceived, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to map topic.");

  // Publisher for matched pose
  matched_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "matched_pose", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing matched poses to /matched_pose");

  mapper_ = new karto::Mapper();

  double correlation_search_space_dimension =
    this->declare_parameter<double>("correlation_search_space_dimension", 0.3);
  mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  double correlation_search_space_resolution =
    this->declare_parameter<double>("correlation_search_space_resolution", 0.1);  //0.01m 10cm
  mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  double correlation_search_space_smear_deviation =
    this->declare_parameter<double>("correlation_search_space_smear_deviation", 0.03);
  mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  double distance_variance_penalty =
    this->declare_parameter<double>("distance_variance_penalty", 0.3);
  mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  double angle_variance_penalty =
    this->declare_parameter<double>("angle_variance_penalty", 0.349);
  mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  double fine_search_angle_offset =
    this->declare_parameter<double>("fine_search_angle_offset", 0.00349);
  mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset =
    this->declare_parameter<double>("coarse_search_angle_offset", M_PI);     // 0.349   360도 간격으로 
  mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution =
    this->declare_parameter<double>("coarse_angle_resolution", 5.0* M_PI/180.0);   // 0.0349 5도 간격으로
  mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  double minimum_angle_penalty =
    this->declare_parameter<double>("minimum_angle_penalty", 0.9);
  mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  double minimum_distance_penalty =
    this->declare_parameter<double>("minimum_distance_penalty", 0.5);
  mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  bool use_response_expansion =
    this->declare_parameter<bool>("use_response_expansion", false);
  mapper_->setParamUseResponseExpansion(use_response_expansion);
}

SlamKarto::~SlamKarto()
{
  for (auto & entry : lasers_) {
    delete entry.second;
  }

  delete mapper_;
  delete m_pCorrelationGrid_;
  delete scanmatcher_;
}

karto::LaserRangeFinder *
SlamKarto::getLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  if (lasers_.find(scan->header.frame_id) == lasers_.end()) {
    RCLCPP_INFO(this->get_logger(), "Creating laser range finder for frame %s", scan->header.frame_id.c_str());

    karto::LaserRangeFinder * laser =
      karto::LaserRangeFinder::CreateLaserRangeFinder();
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);

    lasers_[scan->header.frame_id] = laser;
  }

  return lasers_[scan->header.frame_id];
}

void
SlamKarto::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg)
{
  double yaw = tf2::getYaw(msg->pose.pose.orientation);
  manual_pose_ = karto::Pose2(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    yaw);
  has_manual_pose_ = true;
  
  RCLCPP_INFO(
    this->get_logger(), 
    "Received Initial Pose: x=%.3f, y=%.3f, theta=%.3f", 
    manual_pose_.GetX(), manual_pose_.GetY(), manual_pose_.GetHeading());
}

bool
SlamKarto::getLaserPose(karto::Pose2 & karto_pose, const rclcpp::Time & /*stamp*/, const std::string & /*frame_id*/)
{
  if (!has_manual_pose_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for initial pose from RViz (2D Pose Estimate)...");
    return false;
  }

  // Use the manually set pose directly (assuming laser offset is 0 as requested)
  karto_pose = manual_pose_;
  
  RCLCPP_DEBUG(
    this->get_logger(), "Using manual laser pose: x = %f, y = %f, yaw = %f ",
    karto_pose.GetX(),
    karto_pose.GetY(),
    karto_pose.GetHeading());

  return true;
}

void
SlamKarto::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  if (!scanmatcher_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Scan matcher not ready yet; waiting for a map...");
    return;
  }

  laser_count_++;
  if (throttle_scans_ > 0 && (laser_count_ % throttle_scans_) != 0) {
    return;
  }

  karto::Pose2 laser_pose;
  if (addScan(scan, laser_pose)) {
    RCLCPP_DEBUG(
      this->get_logger(), "added scan at pose: %.3f %.3f %.3f",
      laser_pose.GetX(),
      laser_pose.GetY(),
      laser_pose.GetHeading());
  }
}

bool
SlamKarto::addScan(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  karto::Pose2 & karto_pose)
{
  auto * laser = getLaser(scan);
  if (!laser) {
    RCLCPP_WARN(this->get_logger(), "Failed to create laser device for %s; discarding scan",
      scan->header.frame_id.c_str());
    return false;
  }

  if (!getLaserPose(karto_pose, rclcpp::Time(scan->header.stamp), scan->header.frame_id)) {
    return false;
  }

  if (laser->GetNumberOfRangeReadings() != scan->ranges.size()) {
    RCLCPP_ERROR(this->get_logger(), "Laser reading count mismatch! Expected: %d, Got: %zu. Updating laser settings...",
      laser->GetNumberOfRangeReadings(), scan->ranges.size());
    // Update laser settings to match current scan
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    // Note: LaserRangeFinder calculates NumberOfRangeReadings from these params.
    // If it still doesn't match, we might need to force it or recreate the laser.
  }

  std::vector<kt_double> readings;
  readings.reserve(scan->ranges.size());

  for (const auto & range : scan->ranges) {
    readings.push_back(range);
  }

  RCLCPP_INFO(this->get_logger(), "number of laser points = %zu", readings.size());
  RCLCPP_INFO(this->get_logger(), "laser readings copied is done!");

  auto * range_scan =
    new karto::LocalizedRangeScan(readings, laser);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  karto::Pose2 rMean;
  karto::Matrix3 rCovariance;

  double response = scanmatcher_->MatchScan(range_scan, rMean, rCovariance, false, true);
  RCLCPP_INFO(this->get_logger(), "MatchScan finished. Response: %f", response);

  RCLCPP_INFO(this->get_logger(), "Deleting range_scan...");
  delete range_scan;
  RCLCPP_INFO(this->get_logger(), "range_scan deleted. score %f", response);

  // Publish matched pose
  geometry_msgs::msg::PoseStamped matched_pose_msg;
  matched_pose_msg.header.stamp = scan->header.stamp;  // 직접 사용
  matched_pose_msg.header.frame_id = map_frame_;
  matched_pose_msg.pose.position.x = rMean.GetX();
  matched_pose_msg.pose.position.y = rMean.GetY();
  matched_pose_msg.pose.position.z = 0.0;
  
  // Quaternion 변환
  tf2::Quaternion q;
  q.setRPY(0, 0, rMean.GetHeading());
  matched_pose_msg.pose.orientation = tf2::toMsg(q);
  RCLCPP_INFO(this->get_logger(), "Orientation converted.");
  
  RCLCPP_INFO(this->get_logger(), "Publishing matched pose...");
  matched_pose_pub_->publish(matched_pose_msg);
  RCLCPP_INFO(this->get_logger(), "Matched pose published.");

  return response;
}

void
SlamKarto::mapReceived(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  if (first_map_only_ && first_map_received_) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "map received");
  handleMapMessage(*msg);

  RCLCPP_INFO(this->get_logger(), "map has been handled!");

  first_map_received_ = true;
}

void
SlamKarto::handleMapMessage(const nav_msgs::msg::OccupancyGrid & msg)
{
  convertMap(msg);
}

void
SlamKarto::convertMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  delete m_pCorrelationGrid_;
  m_pCorrelationGrid_ = nullptr;

  RCLCPP_INFO(
    this->get_logger(), "Mapper SmearDeviation: %f",
    mapper_->m_pCorrelationSearchSpaceSmearDeviation->GetValue());
  m_pCorrelationGrid_ = karto::CorrelationGrid::CreateGrid(
    map_msg.info.width,
    map_msg.info.height,
    map_msg.info.resolution,
    mapper_->m_pCorrelationSearchSpaceSmearDeviation->GetValue());

  RCLCPP_INFO(this->get_logger(), "start parse map message!");
  RCLCPP_INFO(this->get_logger(), "map width = %u", static_cast<unsigned int>(map_msg.info.width));
  RCLCPP_INFO(this->get_logger(), "map height = %u", static_cast<unsigned int>(map_msg.info.height));

  int free_cells = 0;
  int occupied_cells = 0;
  int unknown_cells = 0;

  const std::size_t total_cells = map_msg.info.width * map_msg.info.height;
  for (std::size_t i = 0; i < total_cells; ++i) {
    const auto cell = map_msg.data[i];
    if (cell == 0) {
      m_pCorrelationGrid_->GetDataPointer()[i] = GridStates_Free;
      ++free_cells;
    } else if (cell == 100) {
      m_pCorrelationGrid_->GetDataPointer()[i] = GridStates_Occupied;
      ++occupied_cells;
    } else {
      m_pCorrelationGrid_->GetDataPointer()[i] = GridStates_Unknown;
      ++unknown_cells;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Free: %d, Occupied: %d, Unknown: %d", free_cells, occupied_cells, unknown_cells);

  karto::Vector2<kt_double> offset;
  offset.SetX(map_msg.info.origin.position.x);
  offset.SetY(map_msg.info.origin.position.y);

  m_pCorrelationGrid_->GetCoordinateConverter()->SetOffset(offset);

  RCLCPP_INFO(this->get_logger(), "Correlation grid ready");

  if (m_pCorrelationGrid_) {
    delete scanmatcher_;
    scanmatcher_ = nullptr;

    scanmatcher_ = karto::ScanMatcher::Create(
      mapper_,
      mapper_->m_pCorrelationSearchSpaceDimension->GetValue(),
      mapper_->m_pCorrelationSearchSpaceResolution->GetValue(),
      mapper_->m_pCorrelationSearchSpaceSmearDeviation->GetValue(),
      rangeThreshold_,
      m_pCorrelationGrid_);
  }

  RCLCPP_INFO(this->get_logger(), "Grid is ok to use!!!");
}

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamKarto>());
  rclcpp::shutdown();
  return 0;
}
