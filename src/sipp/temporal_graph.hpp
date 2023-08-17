#pragma once

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <trajopt/collision_checker.hpp>
#include <trajopt/typedefs.hpp>
#include <vector>

namespace sipp {

struct TimeInterval {
  typedef int TimeType;

  TimeType start;
  TimeType end;

  bool isOverlapped(const TimeInterval& other) const { return !(end < other.start || other.end < start); }
  TimeInterval intersect(const TimeInterval& other) const;  // call this only if isOverlapped returns true

  bool operator==(const TimeInterval& other) const { return start == other.start && end == other.end; }
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

  int getNumberOfAllSafeIntervals() const;
  std::vector<int> getNumberOfSafeIntervals() const;

  int getNumberOfStates() const;
  const NominalState& getState(int time) const;

 protected:
  double m_dt;
  std::vector<NominalState> m_reference_states;  // Expected states of the robot
  std::vector<NominalState> m_obstacle_states_;  // Known states of the other manipulator
};

struct TemporalGraphNode {
  // Filled during initialization with the information from TemporalCollisionInfo
  int state_index;
  TimeInterval safe_interval;

  // Below are used for search and result construction during runtime
  enum { OPEN, CLOSED, UNDISCOVERED } state;
  TemporalGraphNode* predecessor_ptr;
  int arrival_time;

  bool operator>(const TemporalGraphNode& other) const { return arrival_time > other.arrival_time; }
};

struct TimeStrategyKnot {
  int arrival_time;
  int waiting_duration;

  int state_index;
  Eigen::VectorXd state;  //!< The state of the robot at the given time
};
class TemporalGraph {
 public:
  TemporalGraph(const TemporalCollisionInfo& temporal_collision_info);
  bool validate() const;
  std::vector<TimeStrategyKnot> getStrategy();

 private:
  std::vector<TemporalGraphNode*> getSuccesors(TemporalGraphNode* node);

 private:
  const TemporalCollisionInfo& m_temporal_collision_info;
  std::vector<TemporalGraphNode> m_nodes;
  std::vector<int> m_start_index_of_node;
};

void constructSafeIntervalsFromCollisionTimestamps(const std::vector<int>& collision_timestamps, int max_time,
                                                   std::vector<TimeInterval>& safe_intervals);

std::ostream& operator<<(std::ostream& os, const TimeInterval& interval);
std::ostream& operator<<(std::ostream& os, const std::vector<TimeInterval>& interval);

std::ostream& operator<<(std::ostream& os, const TimeStrategyKnot& knot);
std::ostream& operator<<(std::ostream& os, const std::vector<TimeStrategyKnot>& strategy);
}  // namespace sipp
