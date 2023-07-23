#include "traj_plotter.hpp"

#include <boost/foreach.hpp>
#include <iostream>

#include "osgviewer/osgviewer.hpp"
#include "utils/eigen_conversions.hpp"
using namespace OpenRAVE;
using namespace std;

namespace trajopt {

TrajPlotter::TrajPlotter(OR::EnvironmentBasePtr env, ConfigurationPtr config, const VarArray& trajvars, double dt)
    : m_env(env), m_config(config), m_trajvars(trajvars), m_dt(dt), m_decimation(1.0 / dt) {}

void TrajPlotter::Add(const vector<CostPtr>& costs) {
  BOOST_FOREACH (const CostPtr& cost, costs) {
    if (PlotterPtr plotter = boost::dynamic_pointer_cast<Plotter>(cost)) {
      m_plotters.push_back(plotter);
    }
  }
}
void TrajPlotter::Add(const vector<ConstraintPtr>& constraints) {
  BOOST_FOREACH (const ConstraintPtr& cnt, constraints) {
    if (PlotterPtr plotter = boost::dynamic_pointer_cast<Plotter>(cnt)) {
      m_plotters.push_back(plotter);
    }
  }
}
void TrajPlotter::Add(const vector<PlotterPtr>& plotters) {
  BOOST_FOREACH (const PlotterPtr& plotter, plotters) { m_plotters.push_back(plotter); }
}
void TrajPlotter::AddLink(KinBody::LinkPtr link) {
  m_links.insert(link);
}

void TrajPlotter::Add(PlotterPtr plotter) {
  m_plotters.push_back(plotter);
}

void TrajPlotter::AddAnimation(ConfigurationPtr rad, TrajArray* trajPtr) {
  m_obstacleConfig = rad;
  m_obstacleTrajPtr = trajPtr;
}

void TrajPlotter::OptimizerCallback(OptProb*, DblVec& x) {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(m_env);
  vector<GraphHandlePtr> handles;

  MatrixXd traj = getTraj(x, m_trajvars);
  KinBodyPtr body = m_config->GetBodies().front();

  PlotBodyTrajectory(handles, body, traj);

  BOOST_FOREACH (PlotterPtr& plotter, m_plotters) { plotter->Plot(x, *m_env, handles); }

  viewer->Idle();
}

void TrajPlotter::OptimizerAnimationCallback(OptProb*, DblVec& x) {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(m_env);
  vector<GraphHandlePtr> handles;

  TrajArray traj = getTraj(x, m_trajvars);
  KinBodyPtr body = m_config->GetBodies().front();
  vector<int> joint_inds = m_config->GetJointIndices();

  PlotBodyTrajectory(handles, body, traj);

  BOOST_FOREACH (PlotterPtr& plotter, m_plotters) { plotter->Plot(x, *m_env, handles); }  // Other plot function

  viewer->AnimateKinBody(body, joint_inds, traj, m_dt);

  if (m_obstacleTrajPtr != nullptr && m_obstacleConfig != nullptr) {
    KinBodyPtr body = m_obstacleConfig->GetBodies().front();
    vector<int> joint_inds = m_obstacleConfig->GetJointIndices();

    viewer->AnimateKinBody(body, joint_inds, *m_obstacleTrajPtr, m_dt);
  }

  viewer->Idle();
}

void TrajPlotter::PlotBodyTrajectory(vector<GraphHandlePtr>& handles, KinBodyPtr body, const TrajArray& traj) const {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(m_env);
  vector<TrajArray> linktrajs(m_links.size(), TrajArray(traj.rows(), 3));

  Transform lastTransform;
  for (int i = 0; i < traj.rows(); ++i) {
    m_config->SetDOFValues(toDblVec(traj.row(i)));
    if (m_obstacleConfig != nullptr && m_obstacleTrajPtr != nullptr) {
      m_obstacleConfig->SetDOFValues(toDblVec(m_obstacleTrajPtr->row(i)));
    }
    double squareDist = (lastTransform.inverse() * m_config->GetEETransform()).trans.lengthsqr3();
    if (i == traj.rows() - 1 || squareDist > 0.01) {  // if last point or decimation point
      handles.push_back(viewer->PlotKinBody(body));
      SetTransparency(handles.back(), .2);
      lastTransform = m_config->GetEETransform();
    }
    int iLink = 0;
    BOOST_FOREACH (const KinBody::LinkPtr& link, m_links) {
      OR::Vector p = link->GetTransform().trans;
      linktrajs[iLink].row(i) = Eigen::Vector3d(p.x, p.y, p.z);
      ++iLink;
    }
  }

  BOOST_FOREACH (TrajArray& linktraj, linktrajs) {
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> tmp = linktraj.cast<float>();
    handles.push_back(
        viewer->drawlinestrip(tmp.data(), tmp.rows(), tmp.cols() * sizeof(float), 2, RaveVectorf(1, 0, 0, 1)));
  }
}

}  // namespace trajopt
