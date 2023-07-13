#include <openrave-core.h>
#include <stdio.h>

#include <boost/format.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>
#include <cstring>
#include <sstream>
#include <vector>
using namespace OpenRAVE;
using namespace std;
int main(int argc, char** argv) {
  if (argc < 3) {
    RAVELOG_INFO("ikloader robot iktype\n");
    return 1;
  }

  string robotname = argv[1];
  string iktype = argv[2];
  RaveInitialize(true);                               // start openrave core
  EnvironmentBasePtr penv = RaveCreateEnvironment();  // create the main environment
  {
    // lock the environment to prevent changes
    // EnvironmentMutex::scoped_lock lock(penv->GetMutex());
    // load the scen
    if (!penv->Load("data/pr2test2.env.xml")) {
      RAVELOG_ERROR("failed to load the environment\n");
      return 1;
    }
    vector<RobotBasePtr> robots;
    penv->GetRobots(robots);

    RobotBasePtr probot = robots[0];
    if (!probot) {
      penv->Destroy();
      return 2;
    }
    // penv->Add(probot);
    ModuleBasePtr pikfast = RaveCreateModule(penv, "ikfast");
    penv->Add(pikfast, true, "");
    stringstream ssin, ssout;
    ssin << "LoadIKFastSolver " << probot->GetName() << " " << iktype;
    // if necessary, add free inc for degrees of freedom
    // ssin << " " << 0.04f;
    // get the active manipulator
    probot->SetActiveManipulator("rightarm");
    RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
    if (!pikfast->SendCommand(ssout, ssin)) {
      RAVELOG_ERROR("failed to load iksolver\n");
      penv->Destroy();
      return 1;
    }
    RAVELOG_INFO("testing random ik\n");
    while (1) {
      Transform trans;
      trans.rot = quatFromAxisAngle(Vector(RaveRandomFloat() - 0.5, RaveRandomFloat() - 0.5, RaveRandomFloat() - 0.5));
      trans.trans = Vector(RaveRandomFloat() - 0.5, RaveRandomFloat() - 0.5, RaveRandomFloat() - 0.5) * 2;
      vector<dReal> vsolution;
      if (pmanip->FindIKSolution(IkParameterization(trans), vsolution,
                                 IKFO_IgnoreSelfCollisions | IKFO_IgnoreJointLimits)) {
        stringstream ss;
        ss << "solution is: ";
        for (size_t i = 0; i < vsolution.size(); ++i) {
          ss << vsolution[i] << " ";
        }
        ss << endl;
        RAVELOG_INFO(ss.str());
      } else {
        // could fail due to collisions, etc
      }
    }
  }
  RaveDestroy();  // destroy
  return 0;
}