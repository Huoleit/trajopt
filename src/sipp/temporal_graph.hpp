#pragma once

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <trajopt/collision_checker.hpp>
#include <trajopt/typedefs.hpp>
#include <vector>

namespace sipp {

struct TimeInterval {
  int start;
  int end;
};

struct SphereCollisionGeometry {
  Eigen::Vector3d offset;  //!< The offset of the sphere from the origin of the reference link coordinate
  Eigen::Vector3d center;  //!< The center of the sphere in the world coordinate
  double radius;

  int reference_link_id;  //!< The id of the link coordinate that the offset is defined in

  bool isCollided(const SphereCollisionGeometry& other) const;
};

struct NominalState {
  double time;
  Eigen::VectorXd state;                                      //!< The state of the robot at the given time
  std::vector<SphereCollisionGeometry> collision_geometries;  //!< Collision geometries at the current joint config
  std::vector<TimeInterval> safe_intervals;                   //!< Safe intervals at the current joint config
  std::vector<int> collision_timestamp;                       //!< The timestamp of the collision
};

class RobotCollisionGeometry {
 public:
  RobotCollisionGeometry(trajopt::RobotAndDOFPtr rad);
  void addSphere(const Eigen::Vector3d& offset, double radius, int reference_link_id);
  void addSphere(const Eigen::Vector3d& offset, double radius, std::string reference_link_name);
  /**
   * @brief Create collision geometries at the given state
   * @param[in] state The state of the robot
   * @param[out] collisionGeometries The collision geometries at the given state
   */
  void createCollisionGeometriesAtState(const Eigen::VectorXd& state,
                                        std::vector<SphereCollisionGeometry>& collisionGeometries);

 private:
  trajopt::RobotAndDOFPtr m_rad;
  std::vector<SphereCollisionGeometry> m_spheres;  //!< Collision geometries of the robot
};

class TemporalCollisionInfo {
 public:
  TemporalCollisionInfo(double dt);
  void hatch(trajopt::TrajArray& reference_traj, trajopt::TrajArray& obstacle_traj, RobotCollisionGeometry& robot,
             RobotCollisionGeometry& obstacle);

  const std::vector<NominalState>& getReferenceStates() const { return m_reference_states; }

 private:
  double m_dt;
  std::vector<NominalState> m_reference_states;  // Expected states of the robot
  std::vector<NominalState> m_obstacle_states_;  // Known states of the other manipulator
};

struct TemporalGraphNode {
  double arrival_time;

  double safe_time_from;
  double safe_time_to;
};

class TemporalGraph {
 public:
  TemporalGraph(double dt, double max_time);

  void buildGraph(const trajopt::TrajArray& reference_traj);

 private:
  double dt_;
  double max_time_;

  std::vector<TemporalGraphNode> nodes_;
  std::vector<int> start_index_of_node_;
};

void constructSafeIntervalsFromCollisionTimestamps(const std::vector<int>& collision_timestamps, int max_time,
                                                   std::vector<TimeInterval>& safe_intervals);

std::ostream& operator<<(std::ostream& os, const TimeInterval& interval);
std::ostream& operator<<(std::ostream& os, const std::vector<TimeInterval>& interval);
}  // namespace sipp
