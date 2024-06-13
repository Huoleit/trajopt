#include <gtest/gtest.h>

#include "sipp/temporal_graph.hpp"

using namespace sipp;

void testIntervals(const std::vector<TimeInterval>& intervals, const std::vector<TimeInterval>& ans) {
  ASSERT_EQ(intervals.size(), ans.size()) << "Size is wrong\n"
                                          << "Expect: " << ans << "\n"
                                          << "Actual: " << intervals;
  for (int i = 0; i < intervals.size(); ++i) {
    EXPECT_EQ(intervals[i].start, ans[i].start);
    EXPECT_EQ(intervals[i].end, ans[i].end);
  }
}

TEST(ConstructSafeInterval, collision_timestamps_empty) {
  int max_time = 10;
  std::vector<int> collision_timestamps{};
  std::vector<TimeInterval> safe_intervals;
  constructSafeIntervalsFromCollisionTimestamps(collision_timestamps, max_time, safe_intervals);

  std::vector<TimeInterval> ans{{0, 9}};
  testIntervals(safe_intervals, ans);
}

TEST(ConstructSafeInterval, collision_timestamps_font) {
  int max_time = 10;
  std::vector<int> collision_timestamps{0, 1, 5, 8};
  std::vector<TimeInterval> safe_intervals;
  constructSafeIntervalsFromCollisionTimestamps(collision_timestamps, max_time, safe_intervals);

  std::vector<TimeInterval> ans{{2, 4}, {6, 7}, {9, 9}};
  testIntervals(safe_intervals, ans);
}

TEST(ConstructSafeInterval, collision_timestamps_middle) {
  int max_time = 10;
  std::vector<int> collision_timestamps{1, 5, 8};
  std::vector<TimeInterval> safe_intervals;
  constructSafeIntervalsFromCollisionTimestamps(collision_timestamps, max_time, safe_intervals);

  std::vector<TimeInterval> ans{{0, 0}, {2, 4}, {6, 7}, {9, 9}};
  testIntervals(safe_intervals, ans);
}

TEST(ConstructSafeInterval, collision_timestamps_back) {
  int max_time = 10;
  std::vector<int> collision_timestamps{1, 5, 8, 9};
  std::vector<TimeInterval> safe_intervals;
  constructSafeIntervalsFromCollisionTimestamps(collision_timestamps, max_time, safe_intervals);

  std::vector<TimeInterval> ans{{0, 0}, {2, 4}, {6, 7}};
  testIntervals(safe_intervals, ans);
}