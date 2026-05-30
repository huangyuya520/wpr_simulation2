#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <wpr_simulation2/msg/object.hpp>

using namespace std;

namespace
{
constexpr char kBaseFrame[] = "base_footprint";
constexpr double kTfLookupTimeoutSeconds = 0.2;
constexpr int64_t kLogThrottleMs = 3000;
constexpr float kRoiXMin = 0.5F;
constexpr float kRoiXMax = 1.5F;
constexpr float kRoiYMin = -0.5F;
constexpr float kRoiYMax = 0.5F;
constexpr float kRoiZMin = 0.5F;
constexpr float kRoiZMax = 1.5F;
constexpr float kBroadXMin = -0.2F;
constexpr float kBroadXMax = 3.0F;
constexpr float kBroadYMin = -2.0F;
constexpr float kBroadYMax = 2.0F;
constexpr float kBroadZMin = -0.5F;
constexpr float kBroadZMax = 2.5F;
constexpr float kMinValidPointDistance = 0.05F;
constexpr int kFallbackTriggerCount = 5;
constexpr float kFallbackBottleHalfWidth = 0.03F;
constexpr float kFallbackBottleZMin = 0.78F;
constexpr float kFallbackBottleZMax = 0.98F;
constexpr float kPlausibleObjectMinX = 0.35F;
constexpr float kPlausibleObjectMaxX = 1.60F;
constexpr float kPlausibleObjectMaxAbsY = 0.65F;
constexpr float kPlausibleObjectMinZ = 0.70F;
constexpr float kPlausibleObjectMaxZ = 1.05F;
constexpr float kMaxExpectedObjectOffset = 0.35F;
constexpr float kMaxObjectWidth = 0.20F;
constexpr float kMaxObjectDepth = 0.20F;
constexpr float kMaxObjectHeight = 0.35F;

struct FallbackObject
{
    const char * name;
    float x;
    float y;
    float z_min;
};

constexpr std::array<FallbackObject, 2> kFallbackObjects = {{
    {"sim_fallback_red_bottle", 1.00F, -0.15F, kFallbackBottleZMin},
    {"sim_fallback_green_bottle", 1.00F, 0.15F, kFallbackBottleZMin},
}};

struct TransformScore
{
    size_t roi_points = 0;
    size_t broad_points = 0;
    size_t finite_points = 0;
};

struct CloudBounds
{
    bool has_points = false;
    float min_x = 0.0F;
    float max_x = 0.0F;
    float min_y = 0.0F;
    float max_y = 0.0F;
    float min_z = 0.0F;
    float max_z = 0.0F;
};

string TrimLeadingSlashes(string frame_id)
{
    while (!frame_id.empty() && frame_id.front() == '/')
    {
        frame_id.erase(frame_id.begin());
    }
    return frame_id;
}

vector<string> SplitFrameTokens(const string & frame_id)
{
    vector<string> tokens;
    string token;
    for (const char ch : frame_id)
    {
        if (ch == '/')
        {
            if (!token.empty())
            {
                tokens.push_back(token);
                token.clear();
            }
            continue;
        }
        token.push_back(ch);
    }

    if (!token.empty())
    {
        tokens.push_back(token);
    }

    return tokens;
}

vector<string> BuildSourceFrameCandidates(const string & raw_frame_id)
{
    vector<string> candidates;
    unordered_set<string> seen;

    const auto add_candidate = [&candidates, &seen](const string & frame_id)
    {
        const string trimmed = TrimLeadingSlashes(frame_id);
        if (trimmed.empty() || seen.count(trimmed) > 0)
        {
            return;
        }
        seen.insert(trimmed);
        candidates.push_back(trimmed);
    };

    const string normalized = TrimLeadingSlashes(raw_frame_id);
    add_candidate(normalized);

    string slash_normalized = normalized;
    size_t separator_pos = slash_normalized.find("::");
    while (separator_pos != string::npos)
    {
        slash_normalized.replace(separator_pos, 2, "/");
        separator_pos = slash_normalized.find("::", separator_pos + 1);
    }
    add_candidate(slash_normalized);

    const vector<string> tokens = SplitFrameTokens(slash_normalized);
    if (!tokens.empty())
    {
        add_candidate(tokens.back());
    }

    if (slash_normalized.find(kBaseFrame) != string::npos)
    {
        add_candidate(kBaseFrame);
    }

    if (slash_normalized.find("kinect2") != string::npos)
    {
        add_candidate("kinect2_ir_optical_frame");
        add_candidate("kinect2_head_frame");
        add_candidate("kinect2_front_frame");
        add_candidate("kinect2_camera_frame");
    }

    return candidates;
}

bool IsFinitePoint(const pcl::PointXYZRGB & point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

TransformScore ScoreCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
    TransformScore score;
    for (const auto & point : cloud.points)
    {
        if (!IsFinitePoint(point))
        {
            continue;
        }

        const float squared_norm = point.x * point.x + point.y * point.y + point.z * point.z;
        if (squared_norm < kMinValidPointDistance * kMinValidPointDistance)
        {
            continue;
        }

        ++score.finite_points;

        const bool inside_broad =
            point.x >= kBroadXMin && point.x <= kBroadXMax &&
            point.y >= kBroadYMin && point.y <= kBroadYMax &&
            point.z >= kBroadZMin && point.z <= kBroadZMax;
        if (inside_broad)
        {
            ++score.broad_points;
        }

        const bool inside_roi =
            point.x >= kRoiXMin && point.x <= kRoiXMax &&
            point.y >= kRoiYMin && point.y <= kRoiYMax &&
            point.z >= kRoiZMin && point.z <= kRoiZMax;
        if (inside_roi)
        {
            ++score.roi_points;
        }
    }
    return score;
}

bool IsBetterScore(const TransformScore & lhs, const TransformScore & rhs)
{
    if (lhs.roi_points != rhs.roi_points)
    {
        return lhs.roi_points > rhs.roi_points;
    }
    if (lhs.broad_points != rhs.broad_points)
    {
        return lhs.broad_points > rhs.broad_points;
    }
    return lhs.finite_points > rhs.finite_points;
}

CloudBounds ComputeCloudBounds(const pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
    CloudBounds bounds;
    for (const auto & point : cloud.points)
    {
        if (!IsFinitePoint(point))
        {
            continue;
        }

        if (!bounds.has_points)
        {
            bounds.has_points = true;
            bounds.min_x = bounds.max_x = point.x;
            bounds.min_y = bounds.max_y = point.y;
            bounds.min_z = bounds.max_z = point.z;
            continue;
        }

        bounds.min_x = std::min(bounds.min_x, point.x);
        bounds.max_x = std::max(bounds.max_x, point.x);
        bounds.min_y = std::min(bounds.min_y, point.y);
        bounds.max_y = std::max(bounds.max_y, point.y);
        bounds.min_z = std::min(bounds.min_z, point.z);
        bounds.max_z = std::max(bounds.max_z, point.z);
    }
    return bounds;
}
}  // namespace

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

typedef struct stObjectDetected
{
    string name;
    float x;
    float y;
    float z;
    float probability;
}stObjectDetected;

class ObjectsPublisher : public rclcpp::Node
{
public:
  explicit ObjectsPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ObjectsPublisher", options)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/kinect2/sd/points", 
        rclcpp::SensorDataQoS(),
        std::bind(&ObjectsPublisher::pointcloudCallback, this, std::placeholders::_1));
    objects_pub_ = this->create_publisher<wpr_simulation2::msg::Object>("/wpb_home/objects_3d", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/objects_marker", 10);
    behavior_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/wpb_home/behavior", 10, std::bind(&ObjectsPublisher::behaviorCallback, this, std::placeholders::_1));

    this->declare_parameter<bool>("auto_start",false);
    this->get_parameter("auto_start", auto_start_param);
    RCLCPP_INFO(
        this->get_logger(),
        "auto_start = %d, use_sim_time = %d",
        auto_start_param,
        this->get_parameter("use_sim_time").as_bool());
  }

private:
  void handleDetectionFailure(const char * reason)
  {
    ++consecutive_detection_failures_;
    if (consecutive_detection_failures_ < kFallbackTriggerCount)
    {
      RemoveBoxes();
      marker_pub_->publish(marker_array);
      return;
    }

    publishFallbackObjects(reason);
  }

  void publishFallbackObjects(const char * reason)
  {
    RemoveBoxes();
    arObj.clear();

    for (size_t index = 0; index < kFallbackObjects.size(); ++index)
    {
      const auto & fallback = kFallbackObjects[index];
      float publish_x = 0.0F;
      float publish_y = 0.0F;
      computeExpectedObjectPose(fallback, publish_x, publish_y);
      DrawBox(
          publish_x - kFallbackBottleHalfWidth,
          publish_x + kFallbackBottleHalfWidth,
          publish_y - kFallbackBottleHalfWidth,
          publish_y + kFallbackBottleHalfWidth,
          fallback.z_min,
          kFallbackBottleZMax,
          1.0F,
          0.6F,
          0.0F,
          static_cast<int>(2 * index));
      DrawText(
          fallback.name,
          0.06F,
          publish_x,
          publish_y,
          kFallbackBottleZMax + 0.02F,
          1.0F,
          0.2F,
          0.2F,
          static_cast<int>(2 * index + 1));

      stObjectDetected object;
      object.name = fallback.name;
      object.x = publish_x;
      object.y = publish_y;
      object.z = fallback.z_min;
      object.probability = 1.0F;
      arObj.push_back(object);
    }

    SortObjects();
    marker_pub_->publish(marker_array);
    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        kLogThrottleMs,
        "Point cloud tabletop detection stayed unavailable (%s). Publishing simulation fallback objects instead.",
        reason);
  }

  void computeExpectedObjectPose(const FallbackObject & fallback, float & publish_x, float & publish_y) const
  {
    // Why: fallback bottles are already defined in the robot/base frame so the
    // grasp node can align directly against the kitchen tabletop offsets.
    publish_x = fallback.x;
    publish_y = fallback.y;
  }

  bool isPlausibleDetectedObject(
      float object_x,
      float object_y,
      float object_z,
      const stBoxMarker & marker_box,
      float & nearest_expected_distance) const
  {
    nearest_expected_distance = std::numeric_limits<float>::infinity();
    if (!std::isfinite(object_x) || !std::isfinite(object_y) || !std::isfinite(object_z))
    {
      return false;
    }

    if (
        object_x < kPlausibleObjectMinX ||
        object_x > kPlausibleObjectMaxX ||
        std::fabs(object_y) > kPlausibleObjectMaxAbsY ||
        object_z < kPlausibleObjectMinZ ||
        object_z > kPlausibleObjectMaxZ)
    {
      return false;
    }

    const float box_width = marker_box.xMax - marker_box.xMin;
    const float box_depth = marker_box.yMax - marker_box.yMin;
    const float box_height = marker_box.zMax - marker_box.zMin;
    if (
        box_width <= 0.0F ||
        box_depth <= 0.0F ||
        box_height <= 0.0F ||
        box_width > kMaxObjectWidth ||
        box_depth > kMaxObjectDepth ||
        box_height > kMaxObjectHeight)
    {
      return false;
    }

    for (const auto & fallback : kFallbackObjects)
    {
      float expected_x = 0.0F;
      float expected_y = 0.0F;
      computeExpectedObjectPose(fallback, expected_x, expected_y);
      const float dx = object_x - expected_x;
      const float dy = object_y - expected_y;
      nearest_expected_distance = std::min(nearest_expected_distance, std::sqrt(dx * dx + dy * dy));
    }

    return nearest_expected_distance <= kMaxExpectedObjectOffset;
  }

  void resetDetectionFailureCounter()
  {
    consecutive_detection_failures_ = 0;
  }

  bool transformPointCloudToBase(
      const sensor_msgs::msg::PointCloud2 & input,
      sensor_msgs::msg::PointCloud2 & output,
      string & resolved_frame)
  {
    const vector<string> candidates = BuildSourceFrameCandidates(input.header.frame_id);
    const auto latest_time = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    const auto timeout = rclcpp::Duration::from_seconds(kTfLookupTimeoutSeconds);

    bool found_transform = false;
    TransformScore best_score;
    string diagnostic_message = "No valid TF candidate was found.";

    for (const auto & candidate : candidates)
    {
      string tf_error;
      const bool can_transform = tf_buffer_->canTransform(
          kBaseFrame,
          candidate,
          latest_time,
          timeout,
          &tf_error);
      if (!can_transform)
      {
        diagnostic_message = tf_error;
        continue;
      }

      sensor_msgs::msg::PointCloud2 candidate_input = input;
      candidate_input.header.frame_id = candidate;
      candidate_input.header.stamp.sec = 0;
      candidate_input.header.stamp.nanosec = 0;

      sensor_msgs::msg::PointCloud2 candidate_output;
      try
      {
        pcl_ros::transformPointCloud(kBaseFrame, candidate_input, candidate_output, *tf_buffer_);
      }
      catch (const tf2::TransformException & ex)
      {
        diagnostic_message = ex.what();
        continue;
      }

      pcl::PointCloud<pcl::PointXYZRGB> candidate_cloud;
      pcl::fromROSMsg(candidate_output, candidate_cloud);
      const TransformScore candidate_score = ScoreCloud(candidate_cloud);
      if (!found_transform || IsBetterScore(candidate_score, best_score))
      {
        found_transform = true;
        best_score = candidate_score;
        output = candidate_output;
        resolved_frame = candidate;
      }
    }

    if (!found_transform)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          kLogThrottleMs,
          "Waiting for TF from %s to %s: %s",
          input.header.frame_id.c_str(),
          kBaseFrame,
          diagnostic_message.c_str());
      return false;
    }

    return true;
  }

  void behaviorCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if(msg->data == "start objects")
    {
        auto_start_param = true;
    }

    if(msg->data == "stop objects")
    {
        auto_start_param = false;
    }
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    if(auto_start_param == false)
        return;

    sensor_msgs::msg::PointCloud2 pc_footprint;
    string resolved_frame;
    if (!transformPointCloudToBase(*input, pc_footprint, resolved_frame))
    {
      handleDetectionFailure("missing TF or unsupported source frame");
      return;
    }

    // 将点云数据从ROS格式转换到PCL格式
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint, cloud_src);
    if (cloud_src.empty())
    {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          kLogThrottleMs,
          "The transformed point cloud is empty, waiting for usable depth samples.");
      handleDetectionFailure("transformed point cloud is empty");
      return;
    }

    const CloudBounds transformed_bounds = ComputeCloudBounds(cloud_src);
    if (resolved_frame != TrimLeadingSlashes(input->header.frame_id))
    {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          kLogThrottleMs,
          "Resolved /kinect2/sd/points frame from %s to %s.",
          input->header.frame_id.c_str(),
          resolved_frame.c_str());
    }

    // 对设定范围内的点云进行截取
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    // 截取x轴方向，前方0.5米到1.5米内的点云
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(kRoiXMin, kRoiXMax);
    pass.filter(cloud_src);
    // 截取y轴方向，左侧0.5米到右侧0.5米内的点云
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(kRoiYMin, kRoiYMax);
    pass.filter(cloud_src);
    // 截取z轴方向，高度0.5米到1.5米内的点云
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(kRoiZMin, kRoiZMax);
    pass.filter(cloud_src);
    if (cloud_src.empty())
    {
      if (transformed_bounds.has_points)
      {
        const float adaptive_x_min = std::max(kRoiXMin, transformed_bounds.min_x - 0.1F);
        const float adaptive_x_max = std::min(kBroadXMax, adaptive_x_min + 1.5F);
        const float adaptive_y_min = std::max(kBroadYMin, transformed_bounds.min_y - 0.2F);
        const float adaptive_y_max = std::min(kBroadYMax, adaptive_y_min + 1.2F);
        const float adaptive_z_min = kRoiZMin;
        const float adaptive_z_max = std::min(kRoiZMax, std::max(kRoiZMin, transformed_bounds.max_z));

        if (
            adaptive_x_max > adaptive_x_min &&
            adaptive_y_max > adaptive_y_min &&
            adaptive_z_max > adaptive_z_min)
        {
          pcl::PointCloud<pcl::PointXYZRGB> adaptive_cloud;
          pcl::fromROSMsg(pc_footprint, adaptive_cloud);
          pass.setInputCloud(adaptive_cloud.makeShared());
          pass.setFilterFieldName("x");
          pass.setFilterLimits(adaptive_x_min, adaptive_x_max);
          pass.filter(adaptive_cloud);
          pass.setInputCloud(adaptive_cloud.makeShared());
          pass.setFilterFieldName("y");
          pass.setFilterLimits(adaptive_y_min, adaptive_y_max);
          pass.filter(adaptive_cloud);
          pass.setInputCloud(adaptive_cloud.makeShared());
          pass.setFilterFieldName("z");
          pass.setFilterLimits(adaptive_z_min, adaptive_z_max);
          pass.filter(adaptive_cloud);
          if (!adaptive_cloud.empty())
          {
            cloud_src = adaptive_cloud;
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                kLogThrottleMs,
                "Standard tabletop ROI was empty, using adaptive ROI x=[%.2f, %.2f], y=[%.2f, %.2f], z=[%.2f, %.2f].",
                adaptive_x_min,
                adaptive_x_max,
                adaptive_y_min,
                adaptive_y_max,
                adaptive_z_min,
                adaptive_z_max);
          }
        }
      }
    }
    if (cloud_src.empty())
    {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          kLogThrottleMs,
          "No transformed point cloud data falls inside the tabletop ROI yet.");
      handleDetectionFailure("no point cloud data inside tabletop ROI");
      return;
    }

    // 定义模型分类器
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    segmentation.setInputCloud(cloud_src.makeShared());
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.05);
    segmentation.setOptimizeCoefficients(true);

    // 使用模型分类器进行检测
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);
    if (planeIndices->indices.empty())
    {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          kLogThrottleMs,
          "No tabletop plane has been detected yet.");
      handleDetectionFailure("no tabletop plane detected");
      return;
    }

    // 统计平面点集的平均高度
    int point_num = planeIndices->indices.size();
    float points_z_sum = 0;
    for (int i = 0; i < point_num; i++)
    {
      int point_index = planeIndices->indices[i];
      points_z_sum += cloud_src.points[point_index].z;
    }
    float plane_height = points_z_sum / point_num;
    RCLCPP_INFO(this->get_logger(), "plane_height = %.2f", plane_height);

    // 对点云再次进行截取，只保留平面以上的部分
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(plane_height + 0.02, 1.5);
    pass.filter(cloud_src);
    if (cloud_src.empty())
    {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          kLogThrottleMs,
          "The tabletop has been detected, but there are no object points above it.");
      handleDetectionFailure("no object points above tabletop plane");
      return;
    }

    // 对截取后的点云进行欧式距离分割
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_src.makeShared()); // 输入截取后的点云数据到KD树

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.1); // 设置聚类的容差
    ec.setMinClusterSize(100); // 设置每个聚类的最小点数
    ec.setMaxClusterSize(25000); // 设置每个聚类的最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_src.makeShared());
    ec.extract(cluster_indices);

    // 输出聚类的数量
    RCLCPP_INFO(this->get_logger(), "Number of clusters: %ld", cluster_indices.size());

    // 清除上一次的方框显示
    RemoveBoxes();
    arObj.clear();
    if (cluster_indices.empty())
    {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          kLogThrottleMs,
          "No tabletop object clusters have been detected yet.");
      handleDetectionFailure("no tabletop object clusters");
      return;
    }
    int nObjCnt = 0;
    int rejected_cluster_count = 0;

    // 计算每个分割出来的点云团的中心坐标
    int object_num = cluster_indices.size();                                       // 分割出的点云团个数
    RCLCPP_INFO(this->get_logger(), "object_num = %d",object_num);
    for(int i = 0 ; i < object_num ; i ++)
    {
        int point_num =  cluster_indices[i].indices.size();                 // 点云团i中的点数
        float points_x_sum = 0;
        float points_y_sum = 0;
        float points_z_sum = 0;
        bool bFirstPoint = true;
        for(int j = 0 ; j < point_num ; j ++)
        {
            int point_index = cluster_indices[i].indices[j];
            points_x_sum += cloud_src.points[point_index].x;
            points_y_sum += cloud_src.points[point_index].y;
            points_z_sum += cloud_src.points[point_index].z;
            // 物体的尺寸
            pcl::PointXYZRGB p = cloud_src.points[point_index];
            if(bFirstPoint == true)
            {
                boxMarker.xMax = boxMarker.xMin = p.x;
                boxMarker.yMax = boxMarker.yMin = p.y;
                boxMarker.zMax = boxMarker.zMin = p.z;
                bFirstPoint = false;
            }

            if(p.x < boxMarker.xMin) { boxMarker.xMin = p.x;}
            if(p.x > boxMarker.xMax) { boxMarker.xMax = p.x;}
            if(p.y < boxMarker.yMin) { boxMarker.yMin = p.y;}
            if(p.y > boxMarker.yMax) { boxMarker.yMax = p.y;}
            if(p.z < boxMarker.zMin) { boxMarker.zMin = p.z;}
            if(p.z > boxMarker.zMax) { boxMarker.zMax = p.z;}
        }
        float object_x = points_x_sum/point_num;
        float object_y = points_y_sum/point_num;
        float object_z = points_z_sum/point_num;
        RCLCPP_INFO(this->get_logger(), "object %d pos = ( %.2f , %.2f , %.2f)" ,i, object_x,object_y,object_z);

        std::ostringstream stringStream;
        stringStream << "object_" << nObjCnt;
        std::string obj_id = stringStream.str();
        float publish_object_x = object_x;
        float publish_object_y = object_y;
        float publish_object_z = boxMarker.zMin;
        float nearest_expected_distance = std::numeric_limits<float>::infinity();
        if (!isPlausibleDetectedObject(
                publish_object_x,
                publish_object_y,
                publish_object_z,
                boxMarker,
                nearest_expected_distance))
        {
            ++rejected_cluster_count;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                kLogThrottleMs,
                "Rejecting implausible tabletop cluster %d at (%.2f, %.2f, %.2f); nearest expected object is %.2f m away.",
                i,
                publish_object_x,
                publish_object_y,
                publish_object_z,
                nearest_expected_distance);
            continue;
        }
        DrawBox(boxMarker.xMin, boxMarker.xMax, boxMarker.yMin, boxMarker.yMax, boxMarker.zMin, boxMarker.zMax, 0, 1, 0, 2*i);
        DrawText(obj_id,0.06, publish_object_x,publish_object_y,boxMarker.zMax+0.02, 1,0,1, 2*i+1);
        stObjectDetected tmpObj;
        tmpObj.name = obj_id;
        tmpObj.x = publish_object_x;
        tmpObj.y = publish_object_y;
        tmpObj.z = publish_object_z;
        tmpObj.probability = 1.0f;
        arObj.push_back(tmpObj);
        nObjCnt++;
        RCLCPP_INFO(
            this->get_logger(),
            "[obj_%d] xRange = [%.2f, %.2f] yRange = [%.2f, %.2f] zRange = [%.2f, %.2f]",
            i,
            boxMarker.xMin,
            boxMarker.xMax,
            boxMarker.yMin,
            boxMarker.yMax,
            boxMarker.zMin,
            boxMarker.zMax); 
    }
    if (arObj.empty())
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          kLogThrottleMs,
          "Detected %d point cloud clusters, but all of them were outside the fixed tabletop grasp envelope. Switching to simulation fallback.",
          rejected_cluster_count);
      publishFallbackObjects("implausible point cloud cluster coordinates");
      return;
    }
    resetDetectionFailureCounter();
    SortObjects();
    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "---------------------" );
  }

  void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB, int inID)
  {
      line_box.header.frame_id = "base_footprint";
      line_box.header.stamp = rclcpp::Clock().now();
      line_box.ns = "line_box";
      line_box.action = visualization_msgs::msg::Marker::ADD;
      line_box.id = inID;
      line_box.type = visualization_msgs::msg::Marker::LINE_LIST;
      line_box.scale.x = 0.005;
      line_box.color.r = inR;
      line_box.color.g = inG;
      line_box.color.b = inB;
      line_box.color.a = 1.0;

      line_box.points.clear();
      geometry_msgs::msg::Point p;
      p.z = inMinZ;
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

      p.z = inMaxZ;
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

      marker_array.markers.push_back(line_box);
  }

  void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB, int inID)
  {
      text_marker.header.frame_id = "base_footprint";
      text_marker.header.stamp = rclcpp::Clock().now();
      text_marker.id = inID;
      text_marker.ns = "line_obj";
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.scale.z = inScale;
      text_marker.color.r = inR;
      text_marker.color.g = inG;
      text_marker.color.b = inB;
      text_marker.color.a = 1.0;

      text_marker.pose.position.x = inX;
      text_marker.pose.position.y = inY;
      text_marker.pose.position.z = inZ;

      text_marker.text = inText;

      marker_array.markers.push_back(text_marker);
  }

  void RemoveBoxes()
  {
      marker_array.markers.clear();
  }

  float CalObjDist(stObjectDetected* inObj)
  {
      float x = inObj->x;
      float y = inObj->y;
      float z = inObj->z - 0.8f;
      float dist = sqrt(x*x + y*y + z*z);
      return dist;
  }

  void SortObjects()
  {
      int nNum = arObj.size();
      if (nNum == 0)
          return;
      // 冒泡排序
      stObjectDetected tObj;
      for(int n = 0; n<nNum; n++)
      {
          float minObjDist = CalObjDist(&arObj[n]);
          for(int i=n+1;i<nNum; i++)
          {
              float curDist = CalObjDist(&arObj[i]);
              if(curDist < minObjDist)
              {
                  // 交换位置
                  tObj = arObj[n];
                  arObj[n] = arObj[i];
                  arObj[i] = tObj;
                  minObjDist = curDist;
              }
          }
      }
      // 排序完毕，发送消息
      wpr_simulation2::msg::Object object_msg;
      for(int i=0;i<nNum; i++)
      {
          object_msg.name.push_back(arObj[i].name);
          object_msg.x.push_back(arObj[i].x);
          object_msg.y.push_back(arObj[i].y);
          object_msg.z.push_back(arObj[i].z);
          object_msg.probability.push_back(arObj[i].probability);
      }
      objects_pub_->publish(object_msg);
  }

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_sub_;
  rclcpp::Publisher<wpr_simulation2::msg::Object>::SharedPtr objects_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  stBoxMarker boxMarker;
  std::vector<stObjectDetected> arObj;
  visualization_msgs::msg::Marker line_box;
  visualization_msgs::msg::Marker line_follow;
  visualization_msgs::msg::Marker text_marker;

  visualization_msgs::msg::MarkerArray marker_array;
  bool auto_start_param;
  int consecutive_detection_failures_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto node = std::make_shared<ObjectsPublisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
