#include <openrave-core.h>

#include <iostream>

#include "osgviewer/osgviewer.hpp"
#include "osgviewer/robot_ui.hpp"

using namespace OpenRAVE;
using namespace std;

int main() {
  RaveInitialize(true, OpenRAVE::Level_Info);
  EnvironmentBasePtr env = RaveCreateEnvironment();

  {
    bool success = env->Load("robots/pr2-beta-static.zae");
    FAIL_IF_FALSE(success);
  }

  vector<RobotBasePtr> robots;
  env->GetRobots(robots);
  RobotBasePtr robot = robots[0];
  Transform I;
  I.identity();
  robot->SetTransform(I);

  RobotBase::ManipulatorPtr left_manip = robot->GetManipulator("leftarm");
  RobotBase::ManipulatorPtr right_manip = robot->GetManipulator("rightarm");

  // // IK Start
  // ModuleBasePtr pikfast = RaveCreateModule(env, "ikfast");
  // env->Add(pikfast, true, "");
  // stringstream ssin, ssout;
  // ssin << "LoadIKFastSolver"
  //      << " " << robot->GetName() << " "
  //      << "Transform6D";
  // if (!pikfast->SendCommand(ssout, ssin)) {
  //   RAVELOG_ERROR("failed to load iksolver\n");
  //   env->Destroy();
  //   return 1;
  // }
  // // IK end
  left_manip->SetIkSolver(RaveCreateIkSolver(env, "ikfast_pr2_leftarm"));
  right_manip->SetIkSolver(RaveCreateIkSolver(env, "ikfast_pr2_rightarm"));

  OSGViewerPtr viewer(new OSGViewer(env));
  ManipulatorControl mc_left(left_manip, viewer, true);
  ManipulatorControl mc_right(right_manip, viewer, false);
  DriveControl dc(robot, viewer);
  StatePrinter sp(robot);
  viewer->AddKeyCallback('z', boost::bind(&StatePrinter::PrintAll, &sp));

  viewer->Idle();
}
