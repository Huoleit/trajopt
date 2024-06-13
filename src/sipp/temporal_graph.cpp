#include "temporal_graph.hpp"

#include <algorithm>
#include <queue>

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

std::ostream& operator<<(std::ostream& os, const TimeStrategyKnot& knot) {
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "],");
  os << "\033[34m" << knot.state_index << "(" << (knot.isConfident ? "+" : "-") << ")"
     << "\033[0m: arrive at time \033[33m" << knot.arrival_time << "\033[0m and wait for \033[33m"
     << knot.waiting_duration << "\033[0m steps. Configuration : " << knot.state.transpose().format(CleanFmt);
  return os;
}
std::ostream& operator<<(std::ostream& os, const std::vector<TimeStrategyKnot>& strategy) {
  for (int i = 0; i < strategy.size(); ++i) {
    os << strategy[i];
    if (i != strategy.size() - 1) {
      os << "\n";
    }
  }
  return os;
}

TimeInterval TimeInterval::intersect(const TimeInterval& other) const {  // call this only if isOverlapped returns true
  TimeInterval result;
  result.start = std::max(start, other.start);
  result.end = std::min(end, other.end);
  return result;
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

void constructSafeIntervalsFromCollisionTimestamps(const std::vector<int>& collision_timestamps, int max_num_timestamp,
                                                   std::vector<TimeInterval>& safe_intervals) {
  int current = -1;
  int next;
  for (int i = 0; i < collision_timestamps.size(); ++i, current = next) {
    next = collision_timestamps[i];
    if (next - current > 1) {  // safe interval in the middle
      safe_intervals.push_back({current + 1, next - 1});
    }
  }
  if (current + 1 < max_num_timestamp) {  // last safe interval
    safe_intervals.push_back({current + 1, max_num_timestamp - 1});
  }
}

TemporalCollisionInfo::TemporalCollisionInfo(double dt) : m_dt(dt){};

void TemporalCollisionInfo::bake(const trajopt::TrajArray& reference_traj, const trajopt::TrajArray& obstacle_traj,
                                 RobotCollisionGeometry& robot, RobotCollisionGeometry& obstacle) {
  int N_ref = reference_traj.rows();  // Number of nominal states of the robot
  int N_obs = obstacle_traj.rows();   // Number of states of the obstacle

  m_reference_states.resize(N_ref);
  m_obstacle_states_.resize(N_obs);

  // Construct nominal states with collision geometries information
  for (int i = 0; i < N_ref; ++i) {
    m_reference_states[i].time = m_dt * (double)i;
    m_reference_states[i].state = reference_traj.row(i);
    robot.createCollisionGeometriesAtState(m_reference_states[i].state, m_reference_states[i].collision_geometries);
  }
  // Construct states for obstacle
  for (int i = 0; i < N_obs; ++i) {
    m_obstacle_states_[i].time = m_dt * (double)i;
    m_obstacle_states_[i].state = obstacle_traj.row(i);
    obstacle.createCollisionGeometriesAtState(m_obstacle_states_[i].state, m_obstacle_states_[i].collision_geometries);
  }

  for (int i = 0; i < N_obs; ++i) {  // Obstacle is moving - iterate all possible states for obstacle
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
    constructSafeIntervalsFromCollisionTimestamps(robot_state.collision_timestamp, std::max(N_ref, N_obs),
                                                  robot_state.safe_intervals);
  }
}

int TemporalCollisionInfo::getNumberOfAllSafeIntervals() const {
  std::vector<int> num_of_safe_intervals = getNumberOfSafeIntervals();
  return std::accumulate(num_of_safe_intervals.begin(), num_of_safe_intervals.end(), 0);
}
std::vector<int> TemporalCollisionInfo::getNumberOfSafeIntervals() const {
  std::vector<int> num_of_safe_intervals;
  for (const auto& state : m_reference_states) {
    num_of_safe_intervals.push_back(state.safe_intervals.size());
  }
  return num_of_safe_intervals;
}

const NominalState& TemporalCollisionInfo::getState(int index) const {
  int N = m_reference_states.size();
  if (index < 0 || index >= N) {
    throw std::runtime_error("Index out of range");
  }
  return m_reference_states[index];
}

int TemporalCollisionInfo::getNumberOfStates() const {
  return m_reference_states.size();
}

TemporalGraph::TemporalGraph(const TemporalCollisionInfo& temporal_collision_info)
    : m_temporal_collision_info(temporal_collision_info) {
  int N = temporal_collision_info.getNumberOfAllSafeIntervals();
  m_nodes.reserve(N);
  m_start_index_of_node.reserve(temporal_collision_info.getNumberOfStates() + 1);

  int node_index = 0;
  for (int i = 0; i < temporal_collision_info.getNumberOfStates(); ++i) {
    m_start_index_of_node.push_back(node_index);  // start index of node for state i
    for (const auto& interval : temporal_collision_info.getState(i).safe_intervals) {
      TemporalGraphNode node;
      node.state_index = i;
      node.safe_interval = interval;

      node.state = TemporalGraphNode::UNDISCOVERED;  // initialize state to undiscovered
      node.predecessor_ptr = nullptr;

      m_nodes.push_back(node);
      ++node_index;
    }
  }
  if (temporal_collision_info.getNumberOfStates()) {
    m_start_index_of_node.push_back(N);  // mimic the csc format
  }

  validate();
}

bool TemporalGraph::validate() const {
  if (m_nodes.empty()) {
    throw std::runtime_error("Temporal graph is empty");
  }
  if (m_start_index_of_node.size() < 2 && m_start_index_of_node[1] != 1) {
    throw std::runtime_error("Initial state is not always safe. Cannot find a strategy.");
  }
  if (m_start_index_of_node.back() != m_nodes.size()) {
    throw std::runtime_error("The number of nodes does not match the start index of nodes");
  }
  if (m_start_index_of_node.size() < 2 && m_start_index_of_node.back() - *(m_start_index_of_node.end() - 1) != 1) {
    throw std::runtime_error("Goal state is not always safe. Cannot find a strategy.");
  }
  return true;
}

std::vector<TemporalGraphNode*> TemporalGraph::getSuccesors(TemporalGraphNode* node) {
  std::vector<TemporalGraphNode*> successors;
  for (int i = m_start_index_of_node[node->state_index + 1]; i < m_start_index_of_node[node->state_index + 2]; ++i) {
    successors.push_back(&m_nodes[i]);
  }
  return successors;
}

bool TemporalGraph::getStrategy(std::vector<TimeStrategyKnot>& strategy) {
  auto compare = [](TemporalGraphNode* lhs, TemporalGraphNode* rhs) { return *lhs > *rhs; };
  std::priority_queue<TemporalGraphNode*, std::vector<TemporalGraphNode*>, decltype(compare)> open_queue(compare);
  m_nodes.front().arrival_time = 0;
  m_nodes.front().state = TemporalGraphNode::OPEN;
  open_queue.push(&m_nodes.front());

  while (!open_queue.empty()) {
    // Pop the node with the smallest arrival time
    TemporalGraphNode* current_node = open_queue.top();
    open_queue.pop();

    // Check if have reached the goal. Currently goal is always the last node and is always safe
    current_node->state = TemporalGraphNode::CLOSED;
    if (current_node == &m_nodes.back()) {
      break;
    }

    std::vector<TemporalGraphNode*> successors = getSuccesors(current_node);

    // Possible arrival time interval to reach any successor node
    TimeInterval possible_arrival_interval{.start = current_node->arrival_time + 1,
                                           .end = current_node->safe_interval.end + 1};

    for (TemporalGraphNode* successor : successors) {
      // if the successor is already closed or the arrival time interval does not overlap with the safe interval, skip
      if (successor->state == TemporalGraphNode::CLOSED ||
          !possible_arrival_interval.isOverlapped(successor->safe_interval)) {
        continue;
      }
      TimeInterval::TimeType arrival_time = possible_arrival_interval.intersect(successor->safe_interval).start;
      if (successor->state == TemporalGraphNode::UNDISCOVERED) {
        successor->arrival_time = arrival_time;
        successor->predecessor_ptr = current_node;
        successor->state = TemporalGraphNode::OPEN;
        open_queue.push(successor);
      } else if (successor->state == TemporalGraphNode::OPEN && arrival_time < successor->arrival_time) {
        successor->arrival_time = arrival_time;
        successor->predecessor_ptr = current_node;
      }
    }
  }

  constructStrategy(strategy);

  return m_nodes.back().state == TemporalGraphNode::CLOSED;
}

void TemporalGraph::constructStrategy(std::vector<TimeStrategyKnot>& strategy) {
  strategy.clear();
  strategy.reserve(m_temporal_collision_info.getNumberOfStates());

  // back track to construct the strategy
  auto node_iter = std::find_if(m_nodes.rbegin(), m_nodes.rend(),
                                [](TemporalGraphNode& node) { return node.state == TemporalGraphNode::CLOSED; });
  TemporalGraphNode* current_node;

  for (int i = m_start_index_of_node[node_iter->state_index]; i < m_start_index_of_node[node_iter->state_index + 1];
       ++i) {
    if (m_nodes[i].state == TemporalGraphNode::CLOSED) {
      current_node = &m_nodes[i];
      break;
    }
  }
  int uncertain_from_time = current_node->arrival_time + 1;
  int uncertain_from_index = current_node->state_index + 1;

  int next_arrival_time = current_node->arrival_time + 1;  // we assume that the cost of one movement is 1
  while (current_node != nullptr) {
    TimeStrategyKnot knot;
    knot.arrival_time = current_node->arrival_time;
    knot.waiting_duration = next_arrival_time - current_node->arrival_time - 1;
    knot.isConfident = true;
    knot.state_index = current_node->state_index;
    knot.state = m_temporal_collision_info.getState(knot.state_index).state;
    strategy.push_back(knot);

    next_arrival_time = current_node->arrival_time;
    current_node = current_node->predecessor_ptr;
  }

  std::reverse(strategy.begin(), strategy.end());

  for (int i = uncertain_from_index; i < m_temporal_collision_info.getNumberOfStates(); ++i) {
    TimeStrategyKnot knot;
    knot.arrival_time = uncertain_from_time++;
    knot.waiting_duration = 0;
    knot.isConfident = false;
    knot.state_index = i;
    knot.state = m_temporal_collision_info.getState(knot.state_index).state;
    strategy.push_back(knot);
  }
}

trajopt::TrajArray ConstructTrajArrayFromStrategy(const std::vector<TimeStrategyKnot>& strategy) {
  trajopt::TrajArray traj(strategy.back().arrival_time + 1, strategy.front().state.size());
  int cur_time = 0;
  for (int i = 0; i < strategy.size(); ++i) {
    traj.row(cur_time) = strategy[i].state;
    ++cur_time;
    for (int j = 0; j < strategy[i].waiting_duration; ++j) {
      traj.row(cur_time) = strategy[i].state;
      ++cur_time;
    }
  }
  return traj;
}

}  // namespace sipp