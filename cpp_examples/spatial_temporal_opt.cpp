#include <openrave-core.h>

#include "get_dir.h"
#include "osgviewer/osgviewer.hpp"
#include "osgviewer/robot_ui.hpp"
#include "sco/optimizers.hpp"
#include "sipp/temporal_graph.hpp"
#include "trajopt/collision_checker.hpp"
#include "trajopt/common.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "utils/eigen_conversions.hpp"

using namespace OpenRAVE;
using namespace std;
using namespace trajopt;

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
    bool success = env->Load(getDataPath() + "/abb/model.xml");
    FAIL_IF_FALSE(success);
  }

  viewer.reset(new OSGViewer(env));
  viewer->UpdateSceneData();
  env->AddViewer(viewer);

  RobotBasePtr robot = GetRobot(*env);
  robot->SetDOFValues(DblVec(robot->GetDOF(), 0));
  Transform I;
  I.identity();
  robot->SetTransform(I);

  ProblemConstructionInfo pci(env);
  Json::Value root = readJsonFile(getConfigPath() + "/abb/spatial_temporal.json");
  pci.fromJson(root);
  pci.rad->SetRobotActiveDOFs();
  pci.rad->SetDOFValues(toDblVec(pci.init_info.data.row(0)));
  TrajOptProbPtr prob = ConstructProblem(pci);

  double radius = 0.07;
  sipp::RobotCollisionGeometry robot_collision_geometry(pci.rad);
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, -0.03), radius, "right_link_palm");
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "right_link_sri_ft");
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "right_link_5");
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0.16), radius, "right_link_4");
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0.1), radius, "right_link_4");

  sipp::RobotCollisionGeometry obstacle_collision_geometry(pci.obstacleRad);
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, -0.03), radius, "left_link_palm");
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "left_link_sri_ft");
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "left_link_5");
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0.16), radius, "left_link_4");
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0.1), radius, "left_link_4");

  BasicTrustRegionSQP opt(prob);
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  TrajArray nominal_traj = getTraj(opt.x(), prob->GetVars());
  // sipp::TemporalCollisionInfo t_info(nominal_traj, pci.obstacle_info.data, robot_collision_geometry,
  //                                    obstacle_collision_geometry);

  viewer.reset();
  env.reset();

  RaveDestroy();
}