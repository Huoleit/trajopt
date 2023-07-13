#include <openrave-core.h>

#include <iostream>

#include "osgviewer/osgviewer.hpp"
#include "osgviewer/robot_ui.hpp"

using namespace OpenRAVE;
using namespace std;

int main() {
  RaveInitialize(true, OpenRAVE::Level_Info);
  EnvironmentBasePtr env = RaveCreateEnvironment();

  bool success = env->Load("data/pr2test2.env.xml");
  assert(success);

  vector<RobotBasePtr> robots;
  env->GetRobots(robots);
  RobotBasePtr robot = robots[0];

  robot->SetActiveManipulator("rightarm");
  RobotBase::ManipulatorPtr activeManip = robot->GetActiveManipulator();

  // IK Start
  ModuleBasePtr pikfast = RaveCreateModule(env, "ikfast");
  env->Add(pikfast, true, "");
  stringstream ssin, ssout;
  ssin << "LoadIKFastSolver"
       << " " << robot->GetName() << " "
       << "Transform6D";
  if (!pikfast->SendCommand(ssout, ssin)) {
    RAVELOG_ERROR("failed to load iksolver\n");
    env->Destroy();
    return 1;
  }
  // IK end

  OSGViewerPtr viewer(new OSGViewer(env));
  ManipulatorControl mc(activeManip, viewer);
  DriveControl dc(robot, viewer);
  StatePrinter sp(robot);
  viewer->AddKeyCallback('a', boost::bind(&StatePrinter::PrintAll, &sp));

  viewer->Idle();
}
