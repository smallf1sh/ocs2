#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/ros.h>

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

using namespace ocs2;

namespace {

class MobileManipulatorTrajectoryTarget final {
 public:
  MobileManipulatorTrajectoryTarget(::ros::NodeHandle& nodeHandle, ::ros::NodeHandle& privateNodeHandle)
      : nodeHandle_(nodeHandle),
        privateNodeHandle_(privateNodeHandle) {
    privateNodeHandle_.param<std::string>("robot_name", robotName_, std::string("mobile_manipulator"));
    privateNodeHandle_.param("publish_rate", publishRate_, 5.0);
    privateNodeHandle_.param("time_horizon", timeHorizon_, 3.0);
    privateNodeHandle_.param("time_step", timeStep_, 0.1);

    privateNodeHandle_.param<std::string>("trajectory_type", trajectoryType_, "circle");
    privateNodeHandle_.param("circle_center_x", circleCenterX_, 0.75);
    privateNodeHandle_.param("circle_center_y", circleCenterY_, 0.0);
    privateNodeHandle_.param("circle_center_z", circleCenterZ_, 0.85);
    privateNodeHandle_.param("circle_radius", circleRadius_, 0.12);
    privateNodeHandle_.param("circle_angular_velocity", circleAngularVelocity_, 0.5);

    privateNodeHandle_.param("line_start_x", lineStartX_, 0.55);
    privateNodeHandle_.param("line_start_y", lineStartY_, -0.15);
    privateNodeHandle_.param("line_start_z", lineStartZ_, 0.85);
    privateNodeHandle_.param("line_end_x", lineEndX_, 0.85);
    privateNodeHandle_.param("line_end_y", lineEndY_, 0.15);
    privateNodeHandle_.param("line_end_z", lineEndZ_, 0.85);
    privateNodeHandle_.param("line_period", linePeriod_, 6.0);
    privateNodeHandle_.param("line_repeat", lineRepeat_, true);

    privateNodeHandle_.param("orientation_w", orientationW_, 1.0);
    privateNodeHandle_.param("orientation_x", orientationX_, 0.0);
    privateNodeHandle_.param("orientation_y", orientationY_, 0.0);
    privateNodeHandle_.param("orientation_z", orientationZ_, 0.0);

    targetPublisher_.reset(new TargetTrajectoriesRosPublisher(nodeHandle_, robotName_));
    observationSubscriber_ =
        nodeHandle_.subscribe<ocs2_msgs::mpc_observation>(robotName_ + "_mpc_observation", 1, &MobileManipulatorTrajectoryTarget::observationCallback, this);
  }

  void run() {
    ::ros::Rate rate(publishRate_);
    while (::ros::ok()) {
      ::ros::spinOnce();
      publishTargetTrajectory();
      rate.sleep();
    }
  }

 private:
  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    hasObservation_ = true;
    if (!hasStartTime_) {
      startTime_ = latestObservation_.time;
      hasStartTime_ = true;
    }
  }

  void publishTargetTrajectory() {
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      if (!hasObservation_) {
        ROS_WARN_THROTTLE(2.0, "Waiting for %s_mpc_observation before publishing a trajectory.", robotName_.c_str());
        return;
      }
      observation = latestObservation_;
    }

    const size_t inputDim = observation.input.size();
    const size_t numPoints = std::max<size_t>(2, static_cast<size_t>(std::floor(timeHorizon_ / timeStep_)) + 1);
    const Eigen::Quaterniond desiredOrientation = normalizedOrientation();

    scalar_array_t timeTrajectory;
    vector_array_t stateTrajectory;
    vector_array_t inputTrajectory;
    timeTrajectory.reserve(numPoints);
    stateTrajectory.reserve(numPoints);
    inputTrajectory.reserve(numPoints);

    for (size_t k = 0; k < numPoints; ++k) {
      const scalar_t t = observation.time + static_cast<scalar_t>(k) * timeStep_;
      const Eigen::Vector3d position = samplePosition(t - startTime_);

      vector_t target(7);
      target << position.x(), position.y(), position.z(), desiredOrientation.coeffs();

      timeTrajectory.push_back(t);
      stateTrajectory.push_back(target);
      inputTrajectory.emplace_back(vector_t::Zero(inputDim));
    }

    targetPublisher_->publishTargetTrajectories(TargetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory});
  }

  Eigen::Quaterniond normalizedOrientation() const {
    Eigen::Quaterniond orientation(orientationW_, orientationX_, orientationY_, orientationZ_);
    orientation.normalize();
    return orientation;
  }

  Eigen::Vector3d samplePosition(scalar_t relativeTime) const {
    if (trajectoryType_ == "line") {
      return sampleLine(relativeTime);
    }
    return sampleCircle(relativeTime);
  }

  Eigen::Vector3d sampleCircle(scalar_t relativeTime) const {
    const scalar_t phase = circleAngularVelocity_ * relativeTime;
    return {circleCenterX_ + circleRadius_ * std::cos(phase), circleCenterY_ + circleRadius_ * std::sin(phase), circleCenterZ_};
  }

  Eigen::Vector3d sampleLine(scalar_t relativeTime) const {
    const Eigen::Vector3d start(lineStartX_, lineStartY_, lineStartZ_);
    const Eigen::Vector3d end(lineEndX_, lineEndY_, lineEndZ_);

    if (linePeriod_ <= 0.0) {
      return end;
    }

    scalar_t alpha = 1.0;
    if (lineRepeat_) {
      const scalar_t phase = std::fmod(relativeTime, linePeriod_) / linePeriod_;
      alpha = (phase <= 0.5) ? 2.0 * phase : 2.0 * (1.0 - phase);
    } else {
      alpha = std::min<scalar_t>(1.0, std::max<scalar_t>(0.0, relativeTime / linePeriod_));
    }

    return (1.0 - alpha) * start + alpha * end;
  }

  ::ros::NodeHandle& nodeHandle_;
  ::ros::NodeHandle& privateNodeHandle_;

  std::string robotName_{"mobile_manipulator"};
  std::string trajectoryType_{"circle"};

  double publishRate_{5.0};
  double timeHorizon_{3.0};
  double timeStep_{0.1};

  double circleCenterX_{0.75};
  double circleCenterY_{0.0};
  double circleCenterZ_{0.85};
  double circleRadius_{0.12};
  double circleAngularVelocity_{0.5};

  double lineStartX_{0.55};
  double lineStartY_{-0.15};
  double lineStartZ_{0.85};
  double lineEndX_{0.85};
  double lineEndY_{0.15};
  double lineEndZ_{0.85};
  double linePeriod_{6.0};
  bool lineRepeat_{true};

  double orientationW_{1.0};
  double orientationX_{0.0};
  double orientationY_{0.0};
  double orientationZ_{0.0};

  ::ros::Subscriber observationSubscriber_;
  std::unique_ptr<TargetTrajectoriesRosPublisher> targetPublisher_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
  bool hasObservation_{false};
  bool hasStartTime_{false};
  scalar_t startTime_{0.0};
};

}  // namespace

int main(int argc, char** argv) {
  ::ros::init(argc, argv, "mobile_manipulator_trajectory_target");
  ::ros::NodeHandle nodeHandle;
  ::ros::NodeHandle privateNodeHandle("~");

  MobileManipulatorTrajectoryTarget trajectoryTarget(nodeHandle, privateNodeHandle);
  trajectoryTarget.run();

  return 0;
}
