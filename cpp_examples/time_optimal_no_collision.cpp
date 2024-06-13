#include <openrave-core.h>

#include <Eigen/Core>

#include "get_dir.h"
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
    bool success = env->Load(getDataPath() + "/abb/model.xml");
    FAIL_IF_FALSE(success);
  }
  viewer.reset(new OSGViewer(env));
  env->AddViewer(viewer);
  // viewer->UpdateSceneData();

  RobotBasePtr robot = GetRobot(*env);
  robot->SetDOFValues(DblVec(robot->GetDOF(), 0));

  ProblemConstructionInfo pci(env);
  Json::Value root = readJsonFile(getConfigPath() + "/abb/get_feasible_no_collision.json");
  pci.fromJson(root);
  pci.rad->SetRobotActiveDOFs();
  pci.rad->SetDOFValues(toDblVec(pci.init_info.data.row(0)));
  TrajOptProbPtr prob = ConstructProblem(pci);

  BasicTrustRegionSQP opt(prob);

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  TrajArray traj = getTraj(opt.x(), prob->GetVars());
  viewer->AnimateKinBody(pci.rad->GetRobot(), pci.rad->GetJointIndices(), traj, pci.basic_info.dt);

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "],");
  cout << "traj: \n" << traj.format(CleanFmt) << endl;

  try {
    vector<GraphHandlePtr> axesPtr;
    for (const auto& term : pci.cost_infos) {
      if (PoseCostInfo* poseCostInfo = dynamic_cast<PoseCostInfo*>(term.get())) {
        Vector3d p = poseCostInfo->xyz;
        Vector4d q = poseCostInfo->wxyz;
        axesPtr.push_back(viewer->PlotAxes(toRaveTransform(q, p), 0.1));
        break;
      }
    }

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
