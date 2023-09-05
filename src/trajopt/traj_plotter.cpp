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
  if (link == nullptr) {
    throw std::runtime_error("[TrajPlotter::AddLink] Null link pointer.");
  }
  m_links.insert(link);
}

void TrajPlotter::Add(PlotterPtr plotter) {
  m_plotters.push_back(plotter);
}

void TrajPlotter::AddAnimation(ConfigurationPtr rad, TrajArray traj) {
  m_obstacleConfig = rad;
  m_obstacleTraj = traj;
}

void TrajPlotter::OptimizerCallback(OptProb*, DblVec& x) {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(m_env);
  vector<GraphHandlePtr> handles;

  MatrixXd traj = getTraj(x, m_trajvars);
  KinBodyPtr body = m_config->GetBodies().front();

  PlotBodyTrajectory(handles, body, traj);

  // BOOST_FOREACH (PlotterPtr& plotter, m_plotters) { plotter->Plot(x, *m_env, handles); }

  viewer->Idle();
}

void TrajPlotter::OptimizerAnimationCallback(OptProb*, DblVec& x, bool plotBody) {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(m_env);
  vector<GraphHandlePtr> handles;

  TrajArray traj = getTraj(x, m_trajvars);
  KinBodyPtr body = m_config->GetBodies().front();
  vector<int> joint_inds = m_config->GetJointIndices();

  if (plotBody) PlotBodyTrajectory(handles, body, traj);

  // BOOST_FOREACH (PlotterPtr& plotter, m_plotters) { plotter->Plot(x, *m_env, handles); }  // Other plot function

  viewer->AnimateKinBody(body, joint_inds, traj, m_dt);

  if (m_obstacleConfig != nullptr && m_obstacleTraj.rows() > 0) {
    KinBodyPtr body = m_obstacleConfig->GetBodies().front();
    vector<int> joint_inds = m_obstacleConfig->GetJointIndices();

    viewer->AnimateKinBody(body, joint_inds, m_obstacleTraj, m_dt);
    if (m_obstacleTraj.rows() != traj.rows()) {
      // TODO: osg do not sync time across different animations. If LOOP mode is used and, the animations with different
      // duration will be out of sync. As we assume constant delta time, we just check the length of the trajectories.
      // Find a better way to handle it.
      throw std::runtime_error("[TrajPlotter::OptimizerAnimationCallback]Trajectory lengths do not match");
    }
  }

  viewer->Idle();
}

void TrajPlotter::StrategyAnimationCallback(const TrajArray& traj) {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(m_env);
  vector<GraphHandlePtr> handles;

  KinBodyPtr body = m_config->GetBodies().front();
  vector<int> joint_inds = m_config->GetJointIndices();

  viewer->AnimateKinBody(body, joint_inds, traj, m_dt);

  if (m_obstacleConfig != nullptr && m_obstacleTraj.rows() > 0) {
    KinBodyPtr body = m_obstacleConfig->GetBodies().front();
    vector<int> joint_inds = m_obstacleConfig->GetJointIndices();

    viewer->AnimateKinBody(body, joint_inds, m_obstacleTraj, m_dt);
    if (m_obstacleTraj.rows() != traj.rows()) {
      // TODO: osg do not sync time across different animations. If LOOP mode is used and, the animations with different
      // duration will be out of sync. As we assume constant delta time, we just check the length of the trajectories.
      // Find a better way to handle it.
      throw std::runtime_error("[TrajPlotter::StrategyAnimationCallback]Trajectory lengths do not match");
    }
  }

  viewer->Idle();
}

void TrajPlotter::PlotBodyTrajectory(vector<GraphHandlePtr>& handles, KinBodyPtr body, const TrajArray& traj) const {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(m_env);
  vector<TrajArray> linktrajs(m_links.size(), TrajArray(traj.rows(), 3));

  Transform lastTransform;
  for (int i = 0; i < traj.rows(); ++i) {
    m_config->SetDOFValues(toDblVec(traj.row(i)));
    if (m_obstacleConfig != nullptr && m_obstacleTraj.rows() > 0) {
      m_obstacleConfig->SetDOFValues(toDblVec(m_obstacleTraj.row(i)));
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
