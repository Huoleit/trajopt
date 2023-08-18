#include <gtest/gtest.h>
#include <openrave-core.h>
#include <openrave/openrave.h>

#include <boost/format.hpp>
#include <iostream>
#include <sstream>
#include <string>

#include "sipp/temporal_graph.hpp"
#include "trajopt/rave_utils.hpp"

using namespace OpenRAVE;
using namespace std;
using namespace trajopt;
using namespace sipp;

string test_data_dir() {
  return string(TEST_DATA_DIR);
}

ostream& operator<<(ostream& os, const vector<int>& v) {
  os << "[";
  for (int i = 0; i < v.size(); ++i) {
    os << v[i];
    if (i != v.size() - 1) os << ", ";
  }
  os << "]";
  return os;
}

template <typename T>
string error_string(int i, const vector<T>& expect, const vector<T>& actual) {
  stringstream ss;
  ss << "At time step " << i << ", expect " << expect << ", but got " << actual;
  return ss.str();
}

class MockTemporalCollisionInfo : public TemporalCollisionInfo {
 public:
  MockTemporalCollisionInfo() : TemporalCollisionInfo(0.1) {}
  std::vector<NominalState>& getStatesContainer() { return m_reference_states; }
};

TEST(sipp, sphere_collision_safe_intervals_construction) {
  EnvironmentBasePtr env = RaveCreateEnvironment();
  ASSERT_TRUE(env->Load(test_data_dir() + "sphere.xml"));
  RobotBasePtr sphereBot_1 = env->GetRobot("sphereBot");
  sphereBot_1->SetName("sphereBot_1");

  ASSERT_TRUE(env->Load(test_data_dir() + "sphere.xml"));
  RobotBasePtr sphereBot_2 = env->GetRobot("sphereBot");
  sphereBot_2->SetName("sphereBot_2");

  RobotAndDOFPtr rad1(new RobotAndDOF(sphereBot_1, IntVec(), DOF_X | DOF_Y | DOF_Z, Vector()));
  RobotAndDOFPtr rad2(new RobotAndDOF(sphereBot_2, IntVec(), DOF_X | DOF_Y | DOF_Z, Vector()));

  double radius = 1.0;
  sipp::RobotCollisionGeometry robot_collision_geometry(rad1);
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "sphere_link");

  sipp::RobotCollisionGeometry obstacle_collision_geometry(rad2);
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "sphere_link");

  TrajArray reference_traj(6, 3);
  // clang-format off
  reference_traj << 0, 0, 0,
                    1, 0, 0,// collide
                    2, 0, 0,
                    3, 0, 0,
                    4, 0, 0,// collide
                    5, 0, 0;
  // clang-format on
  TrajArray obstacle_traj(7, 3);
  // clang-format off
  obstacle_traj << 0, 2.2, 0,
                   1, 1.9, 0, 
                   2, 0, 2.1,
                   3, -2, 0,
                   4, 0, 1.9, 
                   5, 0, 3,
                   10,11,10;
  // clang-format on
  sipp::TemporalCollisionInfo temporal_info(0.1);
  temporal_info.hatch(reference_traj, obstacle_traj, robot_collision_geometry, obstacle_collision_geometry);

  vector<vector<TimeInterval>> ans_intervals{{{0, 6}}, {{0, 0}, {2, 6}}, {{0, 6}},
                                             {{0, 6}}, {{0, 3}, {5, 6}}, {{0, 6}}};
  vector<vector<int>> ans_collision_timestamps{{}, {1}, {}, {}, {4}, {}};

  for (int i = 0; i < temporal_info.getNumberOfStates(); ++i) {
    const sipp::NominalState& state = temporal_info.getState(i);

    ASSERT_EQ(state.collision_timestamp.size(), ans_collision_timestamps[i].size())
        << error_string(i, ans_collision_timestamps[i], state.collision_timestamp);

    ASSERT_EQ(state.collision_timestamp, ans_collision_timestamps[i])
        << error_string(i, ans_collision_timestamps[i], state.collision_timestamp);

    ASSERT_EQ(state.safe_intervals.size(), ans_intervals[i].size())
        << error_string(i, ans_intervals[i], state.safe_intervals);

    ASSERT_EQ(state.safe_intervals, ans_intervals[i]) << error_string(i, ans_intervals[i], state.safe_intervals);
  }
}

void validate_strategy(const vector<TimeStrategyKnot>& strategy, const std::vector<NominalState>& states) {
  // Arrival time should be monotonically increasing
  for (int i = 0; i < strategy.size() - 1; ++i) {
    EXPECT_TRUE(strategy[i + 1].arrival_time > strategy[i].arrival_time)
        << boost::format("Arrive time should be monotonically increasing at step %1%") % i;
  }

  // waiting_duration should be non-negative
  for (int i = 0; i < strategy.size(); ++i) {
    EXPECT_TRUE(strategy[i].waiting_duration >= 0)
        << boost::format("Waiting duration should be non-negative at step %1%") % i;
  }

  // Arrival time + waiting duration should be less than the arrival time of next step
  for (int i = 0; i < strategy.size() - 1; ++i) {
    EXPECT_TRUE(strategy[i].arrival_time + strategy[i].waiting_duration < strategy[i + 1].arrival_time)
        << boost::format(
               "Arrival time + waiting duration should be less than the arrival time of next step at step %1%") %
               i;
  }

  // Arrival time + waiting duration should be within the safe interval
  for (int i = 0; i < strategy.size(); ++i) {
    const NominalState& state = states[strategy[i].state_index];
    bool found = false;
    for (int j = 0; j < state.safe_intervals.size(); ++j) {
      const TimeInterval& interval = state.safe_intervals[j];
      if (interval.isInside(strategy[i].arrival_time) &&
          interval.isInside(strategy[i].arrival_time + strategy[i].waiting_duration)) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found)
        << boost::format("Arrival time + waiting duration should be within the safe interval at step %1%") % i << "\n"
        << "Safe intervals: " << state.safe_intervals << "\n"
        << "Arrival time: " << strategy[i].arrival_time << "\n"
        << "Waiting duration: " << strategy[i].waiting_duration;
  }
}

TEST(sipp, graph_search) {
  MockTemporalCollisionInfo info;

  NominalState state_0;
  state_0.safe_intervals = {{0, 10}};
  info.getStatesContainer().push_back(state_0);

  NominalState state_1;
  state_1.safe_intervals = {{0, 5}, {7, 10}};
  info.getStatesContainer().push_back(state_1);

  NominalState state_2;
  state_2.safe_intervals = {{6, 6}};
  info.getStatesContainer().push_back(state_2);

  TemporalGraph graph(info);
  std::vector<TimeStrategyKnot> result = graph.getStrategy();

  validate_strategy(result, info.getStatesContainer());
  cout << result << endl;
}

TEST(sipp, graph_search_no_solution) {
  MockTemporalCollisionInfo info;

  NominalState state_0;
  state_0.safe_intervals = {{0, 10}};
  info.getStatesContainer().push_back(state_0);

  NominalState state_1;
  state_1.safe_intervals = {{0, 5}, {7, 10}};
  info.getStatesContainer().push_back(state_1);

  NominalState state_2;
  state_2.safe_intervals = {{12, 12}};
  info.getStatesContainer().push_back(state_2);

  TemporalGraph graph(info);
  EXPECT_THROW(graph.getStrategy(), std::runtime_error);
}

TEST(sipp, graph_search_switch_path) {
  MockTemporalCollisionInfo info;

  NominalState state_0;
  state_0.safe_intervals = {{0, 10}};
  info.getStatesContainer().push_back(state_0);

  NominalState state_1;
  state_1.safe_intervals = {{0, 5}, {7, 10}};
  info.getStatesContainer().push_back(state_1);

  NominalState state_2;
  state_2.safe_intervals = {{7, 10}};
  info.getStatesContainer().push_back(state_2);

  TemporalGraph graph(info);
  std::vector<TimeStrategyKnot> result = graph.getStrategy();

  validate_strategy(result, info.getStatesContainer());
  cout << result << endl;
}