#include <cmath>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// 将 map 坐标系下的 body 位姿投影到 map 的 XY 平面。
int main(int argc, char** argv)
{
  ros::init(argc, argv, "project_body_to_map_2d");
  ros::NodeHandle node;
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  // 发布频率与动态样本去重状态。
  ros::Rate rate(100);
  ros::Time last_stamp;
  bool published = false;

  while (node.ok())
  {
    // 查询最新的 body 位姿，失败时跳过本轮发布。
    tf::StampedTransform map_to_body;
    try
    {
      listener.lookupTransform("map", "body", ros::Time(0), map_to_body);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "Cannot project body pose in map: %s", ex.what());
      rate.sleep();
      continue;
    }

    // 动态样本只发布一次；零时间戳表示输入链完全静态。
    if (published && !map_to_body.stamp_.isZero() && map_to_body.stamp_ == last_stamp)
    {
      rate.sleep();
      continue;
    }

    // 检查位置与四元数的有效性。
    const double x = map_to_body.getOrigin().x();
    const double y = map_to_body.getOrigin().y();
    tf::Quaternion rotation = map_to_body.getRotation();
    const double norm_squared = rotation.length2();
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(norm_squared) || norm_squared <= 0.0)
    {
      ROS_WARN_THROTTLE(1.0, "Cannot project an invalid body pose in map");
      rate.sleep();
      continue;
    }

    // 保留平面位置与 yaw，移除高度、roll 和 pitch。
    rotation.normalize();
    const double yaw = tf::getYaw(rotation);

    tf::Transform map_to_body_2d;
    map_to_body_2d.setOrigin(tf::Vector3(x, y, 0.0));
    map_to_body_2d.setRotation(tf::createQuaternionFromYaw(yaw));

    // 动态输入沿用采样时间，静态输入使用当前时间。
    const ros::Time stamp = map_to_body.stamp_.isZero()
                                ? ros::Time::now() : map_to_body.stamp_;
    broadcaster.sendTransform(tf::StampedTransform(map_to_body_2d, stamp, "map", "body_2d"));

    last_stamp = map_to_body.stamp_;
    published = true;
    rate.sleep();
  }

  return 0;
}
