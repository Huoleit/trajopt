#include "trajopt/trajectory_linear_constraints.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <iostream>

#include "sco/expr_ops.hpp"
#include "sco/modeling_utils.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/stl_to_string.hpp"

using namespace std;
using namespace sco;
using namespace Eigen;

// #define DBG(expr) cout << #expr << ": " << CSTR(expr) << std::endl

namespace {

static MatrixXd diffAxis0(const MatrixXd& in) {
  return in.middleRows(1, in.rows() - 1) - in.middleRows(0, in.rows() - 1);
}

}  // namespace

namespace trajopt {

//////////// functions ////////////////

JointVelConstraint::JointVelConstraint(const VarArray& vars, const DblVec& limits)
    : Constraint("joint_vel_limits"), vars_(vars), limits_(limits) {}

vector<double> JointVelConstraint::value(const vector<double>& xvec) {
  MatrixXd traj = getTraj(xvec, vars_);
  MatrixXd diff = diffAxis0(traj);
  // cerr << "traj: \n" << traj << endl;
  // cerr << "diff: \n" << diff << endl;
  // cerr << "value: \n" << diff.cwiseAbs().maxCoeff() - limits_.front() << endl;
  diff.array() = diff.array().abs();
  diff.rowwise() -= util::toVectorXd(limits_).transpose();
  return vector<double>(diff.data(), diff.data() + diff.size());
}

ConvexConstraintsPtr JointVelConstraint::convex(const vector<double>& x, Model* model) {
  throw std::runtime_error("not implemented");
  return ConvexConstraintsPtr();
}

double JointVelConstraint::violation(const vector<double>& x) {
  vector<double> cnts_values = value(x);
  return *std::max_element(cnts_values.begin(), cnts_values.end());
}

}  // namespace trajopt
