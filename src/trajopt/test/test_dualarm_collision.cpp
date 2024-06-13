#include <openrave-core.h>

#include <iostream>

#include "osgviewer/osgviewer.hpp"
#include "osgviewer/robot_ui.hpp"
#include "trajopt/collision_checker.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/logging.hpp"
#include "utils/stl_to_string.hpp"
using namespace OpenRAVE;
using namespace std;
using namespace trajopt;
using namespace util;

void PrintCollisions(const vector<Collision>& collisions) {
  LOG_INFO("%ld collisions found\n", collisions.size());
  for (int i = 0; i < collisions.size(); ++i) {
    const Collision& c = collisions[i];
    // LOG_INFO("%d: bodies: %s-%s. normal: %s. ptA: %s. distance: %.3e\n", i, c.linkA->GetName().c_str(),
    //          c.linkB->GetName().c_str(), Str(c.normalB2A).c_str(), Str(c.ptA).c_str(), c.distance);
    LOG_INFO("%d: bodies: %s-%s  type: %s  distance: %.3e\n", i, c.linkA->GetName().c_str(), c.linkB->GetName().c_str(),
             (c.cctype & 2) ? (c.cctype & 1 ? "Between" : "Time1") : (c.cctype & 1 ? "Time0" : "None"), c.distance);
  }
}

// void AnnotateCollisions(const vector<Collision>& collisions, OSGViewer& viewer, vector<GraphHandlePtr>& handles) {
//   for (int i = 0; i < collisions.size(); ++i) {
//     const Collision& c = collisions[i];
//     GraphHandlePtr handle =
//         viewer.draw3dText(to_string(i), float x, float y, float fontsize, const OpenRAVE::Vector& color);
//     handles.push_back(handle);
//   }
// }

int main() {
  RaveInitialize(true, OpenRAVE::Level_Warn);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();

  OSGViewerPtr viewer(new OSGViewer(env));
  env->AddViewer(viewer);

  bool success = env->Load("robots/pr2-beta-static.zae");
  FAIL_IF_FALSE(success);

  CollisionCheckerPtr checker = CreateCollisionChecker(env);
  RobotBasePtr robot = env->GetRobot("pr2");
  FAIL_IF_FALSE(robot);

  RobotBase::ManipulatorPtr left_manip = robot->GetManipulator("leftarm");
  RobotBase::ManipulatorPtr right_manip = robot->GetManipulator("rightarm");
  left_manip->SetIkSolver(RaveCreateIkSolver(env, "ikfast_pr2_leftarm"));
  right_manip->SetIkSolver(RaveCreateIkSolver(env, "ikfast_pr2_rightarm"));

  RobotAndDOFPtr rad(new RobotAndDOF(robot, right_manip->GetArmIndices()));
  RobotAndDOFPtr obstacleRad(new RobotAndDOF(robot, left_manip->GetArmIndices()));

  DblVec dofvals{0.352417, 0.258496, -1, -0.91908, -0.55483, -0.282656, 2.09128};
  DblVec obstacleDofvals{0.101152, 0.232869, 0.6, -0.279225, -1.64525, -0.139862, 2.76289};
  rad->SetDOFValues(dofvals);
  obstacleRad->SetDOFValues(obstacleDofvals);

  vector<KinBody::LinkPtr> links;
  std::vector<int> ind;
  rad->GetAffectedLinks(links, true, ind);

  DriveControl dc(robot, viewer);
  ManipulatorControl mc_left(left_manip, viewer, true);
  ManipulatorControl mc_right(right_manip, viewer, false);

  vector<Collision> collisions;
  vector<OR::GraphHandlePtr> handles;

  while (true) {
    collisions.clear();
    handles.clear();

    checker->LinksVsAll(links, collisions, -1);

    PlotCollisions(collisions, *env, handles, .02);
    checker->PlotCollisionGeometry(handles);
    PrintCollisions(collisions);
    try {
      viewer->Idle();
    } catch (...) {
      break;
    }
  }

  viewer.reset();
  env.reset();
  RaveDestroy();
}