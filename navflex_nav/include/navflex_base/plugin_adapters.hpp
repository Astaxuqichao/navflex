#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/behavior.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navflex_rogmap_core/controller.hpp"
#include "navflex_rogmap_core/global_planner.hpp"
#include "navflex_rogmap_core/recovery.hpp"

namespace navflex_nav
{

class PlannerPluginAdapter
{
public:
  using Ptr = std::shared_ptr<PlannerPluginAdapter>;
  virtual ~PlannerPluginAdapter() = default;
  virtual uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path, std::string & message) = 0;
  virtual bool cancel() = 0;
};

class Nav2PlannerAdapter : public PlannerPluginAdapter
{
public:
  explicit Nav2PlannerAdapter(nav2_core::GlobalPlanner::Ptr plugin)
  : plugin_(std::move(plugin)) {}
  uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path, std::string & message) override
  {
    return plugin_->makePlan(start, goal, path, message);
  }
  bool cancel() override {return plugin_->cancel();}

private:
  nav2_core::GlobalPlanner::Ptr plugin_;
};

class RogMapPlannerAdapter : public PlannerPluginAdapter
{
public:
  explicit RogMapPlannerAdapter(navflex_rogmap_core::GlobalPlanner::Ptr plugin)
  : plugin_(std::move(plugin)) {}
  uint32_t makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path, std::string & message) override
  {
    navflex_rogmap_core::Trajectory3D trajectory;
    const uint32_t result = plugin_->makePlan(start, goal, trajectory, message);
    path = std::move(trajectory.path);
    return result;
  }
  bool cancel() override {return plugin_->cancel();}

private:
  navflex_rogmap_core::GlobalPlanner::Ptr plugin_;
};

class ControllerPluginAdapter
{
public:
  using Ptr = std::shared_ptr<ControllerPluginAdapter>;
  virtual ~ControllerPluginAdapter() = default;
  virtual uint32_t computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    geometry_msgs::msg::TwistStamped & command,
    nav2_core::GoalChecker * goal_checker, std::string & message) = 0;
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;
  virtual bool cancel() = 0;
};

class Nav2ControllerAdapter : public ControllerPluginAdapter
{
public:
  explicit Nav2ControllerAdapter(nav2_core::Controller::Ptr plugin)
  : plugin_(std::move(plugin)) {}
  uint32_t computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    geometry_msgs::msg::TwistStamped & command,
    nav2_core::GoalChecker * goal_checker, std::string & message) override
  {
    return plugin_->computeVelocityCommands(
      pose, velocity, command, goal_checker, message);
  }
  void setPlan(const nav_msgs::msg::Path & path) override {plugin_->setPlan(path);}
  bool cancel() override {return true;}

private:
  nav2_core::Controller::Ptr plugin_;
};

class RogMapControllerAdapter : public ControllerPluginAdapter
{
public:
  explicit RogMapControllerAdapter(navflex_rogmap_core::Controller::Ptr plugin)
  : plugin_(std::move(plugin)) {}
  uint32_t computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    geometry_msgs::msg::TwistStamped & command,
    nav2_core::GoalChecker * goal_checker, std::string & message) override
  {
    return plugin_->computeVelocityCommands(
      pose, velocity, command, goal_checker, message);
  }
  void setPlan(const nav_msgs::msg::Path & path) override
  {
    navflex_rogmap_core::Trajectory3D trajectory;
    trajectory.path = path;
    plugin_->setTrajectory(trajectory);
  }
  bool cancel() override {return plugin_->cancel();}

private:
  navflex_rogmap_core::Controller::Ptr plugin_;
};

class RecoveryPluginAdapter
{
public:
  using Ptr = std::shared_ptr<RecoveryPluginAdapter>;
  virtual ~RecoveryPluginAdapter() = default;
  virtual uint32_t runBehavior(std::string & message) = 0;
  virtual void stop() = 0;
};

class Nav2RecoveryAdapter : public RecoveryPluginAdapter
{
public:
  explicit Nav2RecoveryAdapter(nav2_core::Behavior::Ptr plugin)
  : plugin_(std::move(plugin)) {}
  uint32_t runBehavior(std::string & message) override
  {
    return plugin_->runBehavior(message);
  }
  void stop() override {plugin_->stop();}

private:
  nav2_core::Behavior::Ptr plugin_;
};

class RogMapRecoveryAdapter : public RecoveryPluginAdapter
{
public:
  explicit RogMapRecoveryAdapter(navflex_rogmap_core::Recovery::Ptr plugin)
  : plugin_(std::move(plugin)) {}
  uint32_t runBehavior(std::string & message) override
  {
    return plugin_->runBehavior(message);
  }
  void stop() override {plugin_->stop();}

private:
  navflex_rogmap_core::Recovery::Ptr plugin_;
};

}  // namespace navflex_nav
