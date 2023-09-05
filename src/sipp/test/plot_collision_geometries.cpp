#include <openrave-core.h>

#include <iostream>

#include "osgviewer/osgviewer.hpp"
#include "sipp/temporal_graph.hpp"
#include "trajopt/configuration_space.hpp"
#include "trajopt/rave_utils.hpp"
#include "utils/eigen_conversions.hpp"

using namespace OpenRAVE;
using namespace std;
using namespace trajopt;

RobotAndDOFPtr RADFromName(const string& name, RobotBasePtr robot) {
  vector<int> dof_inds;
  if (RobotBase::ManipulatorPtr manip = GetManipulatorByName(*robot, name)) {
    vector<int> inds = manip->GetArmIndices();
    dof_inds.insert(dof_inds.end(), inds.begin(), inds.end());
  } else {
    throw std::runtime_error("couldn't find manipulator");
  }
  return RobotAndDOFPtr(new RobotAndDOF(robot, dof_inds));
}

int main() {
  EnvironmentBasePtr env;
  OSGViewerPtr viewer;

  RaveInitialize(false, OpenRAVE::Level_Error);
  env = RaveCreateEnvironment();
  env->StopSimulation();

  {
    bool success = env->Load("/workspaces/trajopt/data/abb/model.xml");
    FAIL_IF_FALSE(success);
  }

  viewer.reset(new OSGViewer(env));
  env->AddViewer(viewer);

  RobotBasePtr robot = GetRobot(*env);

  double radius = 0.07;
  RobotAndDOFPtr rad = RADFromName("rightarm", robot);
  sipp::RobotCollisionGeometry robot_collision_geometry(rad);
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, -0.03), radius, "right_link_palm");
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "right_link_sri_ft");
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "right_link_5");
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0.16), radius, "right_link_4");
  robot_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0.1), radius, "right_link_4");

  RobotAndDOFPtr obstacle_rad = RADFromName("leftarm", robot);
  sipp::RobotCollisionGeometry obstacle_collision_geometry(RADFromName("leftarm", robot));
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, -0.03), radius, "left_link_palm");
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "left_link_sri_ft");
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0), radius, "left_link_5");
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0.16), radius, "left_link_4");
  obstacle_collision_geometry.addSphere(Eigen::Vector3d(0, 0, 0.1), radius, "left_link_4");

  sipp::TemporalCollisionInfo temporal_info(0.1);
  TrajArray reference_traj(1, 6);
  reference_traj.setZero();
  TrajArray obstacle_traj(1, 6);
  obstacle_traj.setZero();

  temporal_info.bake(reference_traj, obstacle_traj, robot_collision_geometry, obstacle_collision_geometry);

  rad->SetDOFValues(util::toDblVec(temporal_info.getState(0).state));
  std::vector<OpenRAVE::GraphHandlePtr> handles;
  for (auto& sphere : temporal_info.getState(0).collision_geometries) {
    handles.push_back(
        viewer->PlotSphere(OpenRAVE::Vector(sphere.center.x(), sphere.center.y(), sphere.center.z()), 0.07));
    SetColor(handles.back(), osg::Vec4(1, 0, 0, 0.3));
  }

  viewer->Idle();
}
