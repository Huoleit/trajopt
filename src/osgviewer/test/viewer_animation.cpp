#include <openrave-core.h>

#include <Eigen/Core>

#include "osgviewer/osgviewer.hpp"
using namespace OpenRAVE;
using namespace std;
using namespace Eigen;

int main() {
  RaveInitialize(true, OpenRAVE::Level_Debug);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  bool success = env->Load("data/pr2test2.env.xml");

  vector<RobotBasePtr> robots;
  env->GetRobots(robots);
  RobotBasePtr robot = robots[0];
  assert(success);

  OSGViewer* v = new OSGViewer(env);

  vector<int> joint_inds = robot->GetManipulator("rightarm")->GetArmIndices();

  MatrixXd traj(10, 7);

  // clang-format off
  traj << -1.832, -0.332, -1.011, -1.437, -1.1  , -1.926,  3.074,
          -1.622, -0.152, -0.888, -1.45 , -1.312, -1.742,  3.064,
          -1.411,  0.028, -0.764, -1.463, -1.525, -1.558,  3.055,
          -1.201,  0.208, -0.641, -1.476, -1.737, -1.373,  3.045,
          -0.99 ,  0.388, -0.517, -1.489, -1.949, -1.189,  3.036,
          -0.78 ,  0.567, -0.394, -1.502, -2.162, -1.005,  3.026,
          -0.569,  0.747, -0.27 , -1.515, -2.374, -0.821,  3.017,
          -0.359,  0.927, -0.147, -1.528, -2.586, -0.636,  3.007,
          -0.148,  1.107, -0.023, -1.541, -2.799, -0.452,  2.998,
           0.062,  1.287,  0.1  , -1.554, -3.011, -0.268,  2.988;
  // clang-format on

  v->AnimateKinBody(robot, joint_inds, traj, 0.5);

  v->Idle();

  VectorXd start = traj.row(0);
  VectorXd end = (VectorXd(7) << 0.346724, 0.756668, -1.1, -1.30972, -0.862992, -0.249662, 1.99127).finished();
  MatrixXd traj2(10, 7);
  for (int i = 0; i < traj2.cols(); i++) {
    traj2.col(i) = VectorXd::LinSpaced(10, start[i], end[i]);
  }

  v->AnimateKinBody(robot, joint_inds, traj2, 0.5);

  v->Idle();

  delete v;
}
