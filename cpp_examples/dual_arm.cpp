#include <openrave-core.h>

#include "get_dir.h"
#include "osgviewer/osgviewer.hpp"
#include "osgviewer/robot_ui.hpp"
#include "sco/optimizers.hpp"
#include "trajopt/collision_checker.hpp"
#include "trajopt/common.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "utils/eigen_conversions.hpp"

// Debug
#include <chrono>

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

  RaveInitialize(false, OpenRAVE::Level_Error);
  env = RaveCreateEnvironment();
  env->StopSimulation();

  {
    bool success = env->Load("robots/pr2-beta-static.zae");
    FAIL_IF_FALSE(success);
  }
  // {
  //   bool success = env->Load(getDataPath() + "/table.xml");
  //   FAIL_IF_FALSE(success);
  // }
  viewer.reset(new OSGViewer(env));
  viewer->UpdateSceneData();
  env->AddViewer(viewer);

  RobotBasePtr robot = GetRobot(*env);
  robot->SetDOFValues(DblVec(robot->GetDOF(), 0));
  Transform I;
  I.identity();
  robot->SetTransform(I);

  ProblemConstructionInfo pci(env);
  Json::Value root = readJsonFile(getConfigPath() + "/dual_arm_config/crossarm.json");
  pci.fromJson(root);
  pci.rad->SetRobotActiveDOFs();
  pci.rad->SetDOFValues(toDblVec(pci.init_info.data.row(0)));
  TrajOptProbPtr prob = ConstructProblem(pci);

  BasicTrustRegionSQP opt(prob);
  TrajPlotter plotter(env, pci.rad, prob->GetVars(), pci.basic_info.dt);

  plotter.Add(prob->getCosts());
  plotter.AddLink(robot->GetLink("r_gripper_tool_frame"));
  plotter.AddAnimation(prob->GetObstacleRad(), &prob->GetObstacleRadTraj());
  // opt.addCallback(boost::bind(&TrajPlotter::OptimizerCallback, &plotter, _1, _2));

  opt.initialize(trajToDblVec(prob->GetInitTraj()));

  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  opt.optimize();
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::cout << "Optimization took " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
            << " millis" << std::endl;
  opt.printBenchmarkingResult();

  {
    vector<GraphHandlePtr> axesPtr;
    for (const auto& term : pci.cost_infos) {
      if (PoseCostInfo* poseCostInfo = dynamic_cast<PoseCostInfo*>(term.get())) {
        Vector3d p = poseCostInfo->xyz;
        Vector4d q = poseCostInfo->wxyz;
        axesPtr.push_back(viewer->PlotAxes(toRaveTransform(q, p), 0.1));
        break;
      }
    }
    Transform identity;
    identity.identity();
    axesPtr.push_back(viewer->PlotAxes(identity, 0.1));

    plotter.OptimizerAnimationCallback(prob.get(), opt.x(), false);
  }  // end scope for axesPtr - axes should be removed before viewer is destroyed

  viewer.reset();
  env.reset();

  RaveDestroy();
}
