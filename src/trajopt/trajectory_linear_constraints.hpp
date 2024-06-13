#pragma once

#include "macros.h"
#include "sco/modeling.hpp"
#include "trajopt/common.hpp"

namespace trajopt {

class TRAJOPT_API JointVelConstraint : public Constraint {
 public:
  JointVelConstraint(const VarArray& traj, const DblVec& limits);

  ConstraintType type() override { return INEQ; }

  vector<double> value(const vector<double>& x) override;
  ConvexConstraintsPtr convex(const vector<double>& x, Model* model) override;

  double violation(const vector<double>& x) override;

 private:
  DblVec limits_;
  VarArray vars_;
  QuadExpr expr_;
};
}  // namespace trajopt