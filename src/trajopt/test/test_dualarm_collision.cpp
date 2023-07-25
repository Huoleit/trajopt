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

// void PlotCollisionGeometry(const osgGA::GUIEventAdapter& ea) {
//   if (handles.size() == 0) {
//     cc->PlotCollisionGeometry(handles);
//     vector<Collision> collisions;
//     cc->AllVsAll(collisions);
//   } else {
//     handles.clear();
//   }
// }

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
  RaveInitialize(false, OpenRAVE::Level_Warn);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();

  OSGViewerPtr viewer(new OSGViewer(env));
  env->AddViewer(viewer);

  bool success = env->Load("robots/pr2-beta-static.zae");
  FAIL_IF_FALSE(success);

  CollisionCheckerPtr checker = CreateCollisionChecker(env);
  RobotBasePtr robot = env->GetRobot("pr2");
  FAIL_IF_FALSE(robot);

  RobotAndDOFPtr rad(new RobotAndDOF(robot, robot->GetManipulator("rightarm")->GetArmIndices()));
  RobotAndDOFPtr obstacleRad(new RobotAndDOF(robot, robot->GetManipulator("leftarm")->GetArmIndices()));

  DblVec dofvals{0.352417, 0.258496, -1, -0.91908, -0.55483, -0.282656, 2.09128};
  DblVec obstacleDofvals{0.101152, 0.232869, 0.6, -0.279225, -1.64525, -0.139862, 2.76289};
  rad->SetDOFValues(dofvals);
  obstacleRad->SetDOFValues(obstacleDofvals);

  vector<KinBody::LinkPtr> links;
  std::vector<int> ind;
  rad->GetAffectedLinks(links, true, ind);

  vector<Collision> collisions;
  checker->SetContactDistance(0.01);
  // checker->CastVsAll(*rad, std::vector<KinBody::LinkPtr>{robot->GetLink("r_forearm_link")}, dofvals, dofvals,
  //                    collisions);
  checker->LinkVsAll(*robot->GetLink("r_forearm_link"), collisions, 3);

  vector<OR::GraphHandlePtr> handles;
  cout << "collisions: " << collisions.size() << endl;
  PlotCollisions(collisions, *env, handles, .02);

  checker->PlotCollisionGeometry(handles);
  // checker->PlotCollisionGeometryByLink(.get(), handles);
  PrintCollisions(collisions);

  DriveControl dc(robot, viewer);

  viewer->Idle();

  viewer.reset();
  env.reset();
  RaveDestroy();
}