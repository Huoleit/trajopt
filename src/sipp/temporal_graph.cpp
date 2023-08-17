#include "temporal_graph.hpp"

#include "utils/eigen_conversions.hpp"

namespace sipp {

namespace {
bool isCollided(const std::vector<SphereCollisionGeometry>& A, const std::vector<SphereCollisionGeometry>& B) {
  if (A.empty() || B.empty()) return false;

  for (auto& a : A) {
    for (auto& b : B) {
      if (a.isCollided(b)) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace

std::ostream& operator<<(std::ostream& os, const TimeInterval& interval) {
  os << "[" << interval.start << ", " << interval.end << "]";
  return os;
}
std::ostream& operator<<(std::ostream& os, const std::vector<TimeInterval>& interval) {
  os << "[";
  for (int i = 0; i < interval.size(); ++i) {
    os << interval[i];
    if (i != interval.size() - 1) {
      os << ", ";
    }
  }
  os << "]";
  return os;
}

bool SphereCollisionGeometry::isCollided(const SphereCollisionGeometry& other) const {
  Eigen::Vector3d diff = center - other.center;
  double distance_squared = diff.squaredNorm();
  double sum_of_radii_squared = (radius + other.radius) * (radius + other.radius);
  return distance_squared < sum_of_radii_squared;
}

RobotCollisionGeometry::RobotCollisionGeometry(trajopt::RobotAndDOFPtr rad) : m_rad(rad) {}

void RobotCollisionGeometry::addSphere(const Eigen::Vector3d& offset, double radius, int reference_link_id) {
  SphereCollisionGeometry sphere;
  sphere.offset = offset;
  sphere.radius = radius;
  sphere.reference_link_id = reference_link_id;
  m_spheres.push_back(sphere);
}
void RobotCollisionGeometry::addSphere(const Eigen::Vector3d& offset, double radius, std::string reference_link_name) {
  int reference_link_id = m_rad->GetRobot()->GetLink(reference_link_name)->GetIndex();
  addSphere(offset, radius, reference_link_id);
}

void RobotCollisionGeometry::createCollisionGeometriesAtState(
    const Eigen::VectorXd& state, std::vector<SphereCollisionGeometry>& collisionGeometries) {
  trajopt::Configuration::SaverPtr saver = m_rad->Save();
  collisionGeometries.clear();

  m_rad->SetDOFValues(util::toDblVec(state));
  for (auto& sphere : m_spheres) {
    SphereCollisionGeometry collision_geometry;
    collision_geometry.offset = sphere.offset;
    collision_geometry.radius = sphere.radius;
    collision_geometry.reference_link_id = sphere.reference_link_id;

    OpenRAVE::Vector offset(sphere.offset.x(), sphere.offset.y(), sphere.offset.z());
    OpenRAVE::Transform transform = m_rad->GetRobot()->GetLinks()[sphere.reference_link_id]->GetTransform();
    OpenRAVE::Vector origin_in_world = transform * offset;
    collision_geometry.center = Eigen::Vector3d(origin_in_world.x, origin_in_world.y, origin_in_world.z);

    collisionGeometries.push_back(collision_geometry);
  }
}

void constructSafeIntervalsFromCollisionTimestamps(const std::vector<int>& collision_timestamps, int max_time,
                                                   std::vector<TimeInterval>& safe_intervals) {
  int current = -1;
  int next;
  for (int i = 0; i < collision_timestamps.size(); ++i, current = next) {
    next = collision_timestamps[i];
    if (next - current > 1) {  // safe interval in the middle
      safe_intervals.push_back({current + 1, next - 1});
    }
  }
  if (current + 1 < max_time) {  // last safe interval
    safe_intervals.push_back({current + 1, max_time - 1});
  }
}

TemporalCollisionInfo::TemporalCollisionInfo(double dt) : m_dt(dt){};

void TemporalCollisionInfo::hatch(trajopt::TrajArray& reference_traj, trajopt::TrajArray& obstacle_traj,
                                  RobotCollisionGeometry& robot, RobotCollisionGeometry& obstacle) {
  if (reference_traj.rows() != obstacle_traj.rows()) {
    throw std::runtime_error(
        "The number of rows in the reference trajectory and the obstacle trajectory must be the same");
  }

  int N = reference_traj.rows();

  m_reference_states.resize(N);
  m_obstacle_states_.resize(N);

  // Construct nominal states with collision geometries information
  for (int i = 0; i < N; ++i) {
    m_reference_states[i].time = m_dt;
    m_reference_states[i].state = reference_traj.row(i);
    robot.createCollisionGeometriesAtState(m_reference_states[i].state, m_reference_states[i].collision_geometries);

    m_obstacle_states_[i].time = m_dt;
    m_obstacle_states_[i].state = obstacle_traj.row(i);
    obstacle.createCollisionGeometriesAtState(m_obstacle_states_[i].state, m_obstacle_states_[i].collision_geometries);
  }

  for (int i = 0; i < N; ++i) {  // Obstacle is moving - iterate all possible states for obstacle
    auto& obstacle_state = m_obstacle_states_[i];
    // Check which robot state is collided with the obstacle at time i
    for (auto& robot_state : m_reference_states) {
      if (isCollided(robot_state.collision_geometries, obstacle_state.collision_geometries)) {
        robot_state.collision_timestamp.push_back(i);
        obstacle_state.collision_timestamp.push_back(i);
      }
    }
  }

  // Construct safe intervals for each state
  for (auto& robot_state : m_reference_states) {
    constructSafeIntervalsFromCollisionTimestamps(robot_state.collision_timestamp, N, robot_state.safe_intervals);
  }
}
}  // namespace sipp