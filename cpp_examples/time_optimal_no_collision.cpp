#include <openrave-core.h>

#include <Eigen/Core>

#include "get_data_dir.h"
#include "osgviewer/osgviewer.hpp"
#include "sco/optimizers.hpp"
#include "trajopt/common.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "utils/eigen_conversions.hpp"

// Debug
#include "utils/stl_to_string.hpp"

using namespace OpenRAVE;
using namespace std;
using namespace trajopt;

#define DBG(expr) cout << #expr << ": " << CSTR(expr) << std::endl

Json::Value readJsonFile(const std::string& fname) {
  Json::Value root;
  Json::Reader reader;
  std::ifstream fh(fname.c_str());
  bool parse_success = reader.parse(fh, root);
  if (!parse_success) throw std::runtime_error("failed to parse " + fname);
  return root;
}

int main() {
  EnvironmentBasePtr env;
  OSGViewerPtr viewer;

  RaveInitialize(false, OpenRAVE::Level_Warn);
  env = RaveCreateEnvironment();
  env->StopSimulation();

  {
    bool success = env->Load("robots/pr2-beta-static.zae");
    FAIL_IF_FALSE(success);
  }
  viewer.reset(new OSGViewer(env));
  env->AddViewer(viewer);
  // viewer->UpdateSceneData();

  RobotBasePtr robot = GetRobot(*env);
  robot->SetDOFValues(DblVec(robot->GetDOF(), 0));

  ProblemConstructionInfo pci(env);
  Json::Value root = readJsonFile(getDataPath() + "/get_feasible_no_collision.json");
  pci.fromJson(root);
  pci.rad->SetRobotActiveDOFs();
  pci.rad->SetDOFValues(toDblVec(pci.init_info.data.row(0)));
  TrajOptProbPtr prob = ConstructProblem(pci);

  BasicTrustRegionSQP opt(prob);

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  TrajArray traj = getTraj(opt.x(), prob->GetVars());
  viewer->AnimateKinBody(pci.rad->GetRobot(), pci.rad->GetJointIndices(), traj, pci.basic_info.dt);
  cout << "traj: \n" << traj << endl;

  try {
    viewer->Idle();
  } catch (...) {
    viewer.reset();
    env.reset();

    RaveDestroy();
  }
  viewer.reset();
  env.reset();

  RaveDestroy();
}
