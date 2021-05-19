#include <discrepancy_planner/util.h>

namespace discrepancy_planner {

Eigen::Quaterniond RPYToQuaternion(double roll, double pitch, double yaw) {
  return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

void QuaternionToRPY(double x, double y, double z, double w, double& roll,
                     double& pitch, double& yaw) {
  Eigen::Quaterniond q(w, x, y, z);
  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  roll = euler[0];
  pitch = euler[1];
  yaw = euler[2];
}

bool YawForZeroRollAndPitch(double roll, double pitch, double yaw,
                            double& yaw_out, double threshold) {
  auto actual_rot = RPYToQuaternion(roll, pitch, yaw);

  bool minimize_angle_error = (threshold < 0);
  double min_angle_error = 100.0;
  double min_angle_error_yaw = yaw;

  double candidate_yaw = yaw;
  auto candidate_rot = RPYToQuaternion(0, 0, candidate_yaw);
  auto diff = actual_rot.inverse() * candidate_rot;
  double angle =
    angles::normalize_angle(2 * std::atan2(diff.vec().norm(), diff.w()));
  if (minimize_angle_error && std::abs(angle) < min_angle_error) {
    min_angle_error = std::abs(angle);
    min_angle_error_yaw = candidate_yaw;
  } else if (!minimize_angle_error && std::abs(angle) < threshold) {
    yaw_out = angles::normalize_angle(candidate_yaw);
    return true;
  }

  candidate_yaw = yaw + M_PI;
  candidate_rot = RPYToQuaternion(0, 0, candidate_yaw);
  diff = actual_rot.inverse() * candidate_rot;
  angle = angles::normalize_angle(2 * std::atan2(diff.vec().norm(), diff.w()));
  if (minimize_angle_error && std::abs(angle) < min_angle_error) {
    min_angle_error = std::abs(angle);
    min_angle_error_yaw = candidate_yaw;
  } else if (!minimize_angle_error && std::abs(angle) < threshold) {
    yaw_out = angles::normalize_angle(candidate_yaw);
    return true;
  }

  candidate_yaw = yaw - M_PI;
  candidate_rot = RPYToQuaternion(0, 0, candidate_yaw);
  diff = actual_rot.inverse() * candidate_rot;
  angle = angles::normalize_angle(2 * std::atan2(diff.vec().norm(), diff.w()));
  if (minimize_angle_error && std::abs(angle) < min_angle_error) {
    min_angle_error = std::abs(angle);
    min_angle_error_yaw = candidate_yaw;
  } else if (!minimize_angle_error && std::abs(angle) < threshold) {
    yaw_out = angles::normalize_angle(candidate_yaw);
    return true;
  }

  if (minimize_angle_error) {
    yaw_out = angles::normalize_angle(min_angle_error_yaw);
    return true;
  }

  return false;
}

}  // namespace discrepancy_planner
