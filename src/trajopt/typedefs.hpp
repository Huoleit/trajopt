#pragma once
#include <openrave/openrave.h>

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <map>
#include <vector>

#include "macros.h"
#include "sco/modeling.hpp"
#include "utils/basic_array.hpp"

namespace trajopt {

namespace OR = OpenRAVE;
using OR::KinBody;
using OR::RobotBase;
using std::map;
using std::vector;
using namespace sco;
using namespace util;

typedef BasicArray<Var> VarArray;
typedef BasicArray<AffExpr> AffArray;
typedef BasicArray<Cnt> CntArray;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;

typedef vector<double> DblVec;
typedef vector<int> IntVec;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

/**
Interface for objects that know how to plot themselves given solution vector x
*/
class Plotter {
 public:
  virtual void Plot(const DblVec& x, OR::EnvironmentBase&, std::vector<OR::GraphHandlePtr>& handles) = 0;
  virtual ~Plotter() {}
};
typedef boost::shared_ptr<Plotter> PlotterPtr;

}  // namespace trajopt
