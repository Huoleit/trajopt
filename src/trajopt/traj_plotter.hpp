#pragma once
#include "trajopt/common.hpp"
namespace trajopt {

struct TRAJOPT_API TrajPlotter {
  OpenRAVE::EnvironmentBasePtr m_env;
  ConfigurationPtr m_config;
  VarArray m_trajvars;
  vector<PlotterPtr> m_plotters;
  std::set<KinBody::LinkPtr> m_links;  // links for which we'll plot the trajectory
  TrajArray m_obstacleTraj;            // trajectory of the obstacle
  ConfigurationPtr m_obstacleConfig;

  int m_decimation;  // Plot every m_decimation points
  double m_dt;

  TrajPlotter(OR::EnvironmentBasePtr env, ConfigurationPtr config, const VarArray& trajvars, double dt = 0.1);
  void Add(const vector<CostPtr>& costs);
  void Add(const vector<ConstraintPtr>& constraints);
  void Add(const vector<PlotterPtr>& plotters);
  void Add(PlotterPtr plotter);
  void AddLink(OpenRAVE::KinBody::LinkPtr link);
  void AddAnimation(ConfigurationPtr rad, TrajArray trajPtr);
  void OptimizerCallback(OptProb*, DblVec& x);
  void OptimizerAnimationCallback(OptProb*, DblVec& x, bool plotBody = true);
  void StrategyAnimationCallback(const TrajArray& traj);
  void SetDecimation(int dec) { m_decimation = dec; }

 private:
  void PlotBodyTrajectory(std::vector<OpenRAVE::GraphHandlePtr>& handles, OpenRAVE::KinBodyPtr body,
                          const TrajArray& traj) const;
};
typedef boost::shared_ptr<TrajPlotter> TrajPlotterPtr;

}  // namespace trajopt
