#include "navflex_base/navflex_execution_base.h"

namespace navflex_nav
{
NavflexExecutionBase::NavflexExecutionBase(
  const std::string & name,
  const navflex_utility::RobotInformation::ConstPtr & robot_info,
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
: should_exit_(false),
  outcome_(255),
  cancel_(false),
  name_(name),
  robot_info_(robot_info),
  node_(node) {}

NavflexExecutionBase::~NavflexExecutionBase()
{
  if (thread_.joinable()) {
    // if the user forgets to call stop(), we have to kill it
    stop();
    thread_.join();
  }
}

bool NavflexExecutionBase::start()
{
  if (thread_.joinable()) {
    // if the user forgets to call stop(), we have to kill it
    stop();
    thread_.join();
  }

  should_exit_ = false;
  thread_ = std::thread(&NavflexExecutionBase::run, this);
  return true;
}

void NavflexExecutionBase::stop()
{
  RCLCPP_WARN_STREAM(
    node_->get_logger(),
    "Try to stop the plugin \""
      << name_ << "\" rigorously by notifying the thread!");

  {
    // Set the exit flag in a critical section
    std::unique_lock<std::mutex> lock(should_exit_mutex_);
    should_exit_ = true;
  }
  // Wake any thread waiting in waitForStateUpdate() so it notices should_exit_
  condition_.notify_all();
}

void NavflexExecutionBase::join()
{
  if (thread_.joinable()) {thread_.join();}
}

std::cv_status NavflexExecutionBase::waitForStateUpdate(
  std::chrono::microseconds const & duration)
{
  std::unique_lock<std::mutex> lock(state_wait_mutex_);
  return condition_.wait_for(lock, duration);
}

uint32_t NavflexExecutionBase::getOutcome() const {return outcome_.load();}

std::string NavflexExecutionBase::getMessage() const
{
  std::lock_guard<std::mutex> lock(message_mtx_);
  return message_;
}

const std::string & NavflexExecutionBase::getName() const {return name_;}

void NavflexExecutionBase::setOutcome(uint32_t outcome)
{
  outcome_.store(outcome);
}

void NavflexExecutionBase::setMessage(const std::string & message)
{
  std::lock_guard<std::mutex> lock(message_mtx_);
  message_ = message;
}

void NavflexExecutionBase::setOutcomeAndMessage(
  uint32_t outcome, const std::string & message)
{
  outcome_.store(outcome);
  setMessage(message);
}

}  // namespace navflex_nav
