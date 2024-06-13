#include "trajopt/collision_terms.hpp"

#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>

#include "sco/expr_ops.hpp"
#include "sco/expr_vec_ops.hpp"
#include "sco/modeling_utils.hpp"
#include "sco/sco_common.hpp"
#include "trajopt/collision_checker.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/logging.hpp"
#include "utils/stl_to_string.hpp"
using namespace OpenRAVE;
using namespace sco;
using namespace util;
using namespace std;

namespace trajopt {

void CollisionsToDistances(const vector<Collision>& collisions, const Link2Int& m_link2ind, DblVec& dists) {
  // Note: this checking (that the links are in the list we care about) is probably unnecessary
  // since we're using LinksVsAll
  dists.clear();
  dists.reserve(collisions.size());
  BOOST_FOREACH (const Collision& col, collisions) {
    Link2Int::const_iterator itA = m_link2ind.find(col.linkA);
    Link2Int::const_iterator itB = m_link2ind.find(col.linkB);
    if (itA != m_link2ind.end() || itB != m_link2ind.end()) {
      dists.push_back(col.distance);
    }
  }
}

void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad, const Link2Int& link2ind,
                                     const VarVector& vars, const DblVec& dofvals, vector<AffExpr>& exprs,
                                     bool isTimestep1) {
  exprs.clear();
  exprs.reserve(collisions.size());
  rad.SetDOFValues(dofvals);  // since we'll be calculating jacobians
  BOOST_FOREACH (const Collision& col, collisions) {
    AffExpr dist(col.distance);
    Link2Int::const_iterator itA = link2ind.find(col.linkA);
    if (itA != link2ind.end()) {
      VectorXd dist_grad = toVector3d(col.normalB2A).transpose() * rad.PositionJacobian(itA->second, col.ptA);
      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
    }
    Link2Int::const_iterator itB = link2ind.find(col.linkB);
    if (itB != link2ind.end()) {
      VectorXd dist_grad =
          -toVector3d(col.normalB2A).transpose() *
          rad.PositionJacobian(itB->second, (isTimestep1 && (col.cctype == CCType_Between)) ? col.ptB1 : col.ptB);
      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
    }
    if (itA != link2ind.end() || itB != link2ind.end()) {
      exprs.push_back(dist);
    }
  }
  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}

void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad, const Link2Int& link2ind,
                                     const VarVector& vars0, const VarVector& vars1, const DblVec& vals0,
                                     const DblVec& vals1, vector<AffExpr>& exprs) {
  vector<AffExpr> exprs0, exprs1;
  CollisionsToDistanceExpressions(collisions, rad, link2ind, vars0, vals0, exprs0, false);
  CollisionsToDistanceExpressions(collisions, rad, link2ind, vars1, vals1, exprs1, true);

  exprs.resize(exprs0.size());
  for (int i = 0; i < exprs0.size(); ++i) {
    exprScale(exprs0[i], (1 - collisions[i].time));
    exprScale(exprs1[i], collisions[i].time);
    exprs[i] = AffExpr(0);
    exprInc(exprs[i], exprs0[i]);
    exprInc(exprs[i], exprs1[i]);
    cleanupAff(exprs[i]);
  }
}

inline size_t hash(const DblVec& x) {
  return boost::hash_range(x.begin(), x.end());
}

void CollisionEvaluator::GetCollisionsCached(const DblVec& x, vector<Collision>& collisions) {
  double key = hash(getDblVec(x, GetVars()));
  vector<Collision>* it = m_cache.get(key);
  if (it != NULL) {
    LOG_DEBUG("using cached collision check\n");
    collisions = *it;
  } else {
    LOG_DEBUG("not using cached collision check\n");
    CalcCollisions(x, collisions);
    m_cache.put(key, collisions);
  }
}

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars)
    : m_env(rad->GetEnv()),
      m_cc(CollisionChecker::GetOrCreate(*m_env)),
      m_rad(rad),
      m_vars(vars),
      m_link2ind(),
      m_links(),
      m_filterMask(-1) {
  vector<KinBody::LinkPtr> links;
  vector<int> inds;
  rad->GetAffectedLinks(m_links, true, inds);
  for (int i = 0; i < m_links.size(); ++i) {
    m_link2ind[m_links[i].get()] = inds[i];
  }
  // TODO add argument
}

void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals = getDblVec(x, m_vars);
  m_rad->SetDOFValues(dofvals);
  m_cc->LinksVsAll(m_links, collisions, m_filterMask);
}

void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, m_link2ind, dists);
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  DblVec dofvals = getDblVec(x, m_vars);
  CollisionsToDistanceExpressions(collisions, *m_rad, m_link2ind, m_vars, dofvals, exprs, false);
}

////////////////////////////////////////

SingleTimestepDualArmCollisionEvaluator::SingleTimestepDualArmCollisionEvaluator(ConfigurationPtr rad,
                                                                                 const VarVector& vars,
                                                                                 ConfigurationPtr obstacleRad,
                                                                                 const DblVec& obstacleRadConfiguration)
    : m_env(rad->GetEnv()),
      m_cc(CollisionChecker::GetOrCreate(*m_env)),
      m_rad(rad),
      m_vars(vars),
      m_link2ind(),
      m_links(),
      m_filterMask(-1),
      m_obstacleRad(obstacleRad),
      m_obstacleConfiguration(obstacleRadConfiguration) {
  vector<int> inds;
  m_rad->GetAffectedLinks(m_links, true, inds);
  for (int i = 0; i < m_links.size(); ++i) {
    m_link2ind[m_links[i].get()] = inds[i];
  }
}

void SingleTimestepDualArmCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals = getDblVec(x, m_vars);
  m_rad->SetDOFValues(dofvals);
  m_obstacleRad->SetDOFValues(m_obstacleConfiguration);
  m_cc->LinksVsAll(m_links, collisions, m_filterMask);
}
void SingleTimestepDualArmCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, m_link2ind, dists);
}
void SingleTimestepDualArmCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  DblVec dofvals = getDblVec(x, m_vars);
  CollisionsToDistanceExpressions(collisions, *m_rad, m_link2ind, m_vars, dofvals, exprs, false);
}

void SingleTimestepDualArmCollisionEvaluator::GetCollisionsCached(const DblVec& x, vector<Collision>& collisions) {
  DblVec concatenate(x);
  concatenate.insert(concatenate.end(), m_obstacleConfiguration.begin(), m_obstacleConfiguration.end());
  Base::GetCollisionsCached(concatenate, collisions);
}

////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1)
    : m_env(rad->GetEnv()),
      m_cc(CollisionChecker::GetOrCreate(*m_env)),
      m_rad(rad),
      m_vars0(vars0),
      m_vars1(vars1),
      m_link2ind(),
      m_links() {
  vector<KinBody::LinkPtr> links;
  vector<int> inds;
  rad->GetAffectedLinks(m_links, true, inds);
  for (int i = 0; i < m_links.size(); ++i) {
    m_link2ind[m_links[i].get()] = inds[i];
  }
}

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);
  m_rad->SetDOFValues(dofvals0);
  m_cc->CastVsAll(*m_rad, m_links, dofvals0, dofvals1, collisions);
}
void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);
  CollisionsToDistanceExpressions(collisions, *m_rad, m_link2ind, m_vars0, m_vars1, dofvals0, dofvals1, exprs);
}
void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, m_link2ind, dists);
}

////////////////////////////////////////

CastDualArmCollisionEvaluator::CastDualArmCollisionEvaluator(ConfigurationPtr primaryRad, const VarVector& vars0,
                                                             const VarVector& vars1, ConfigurationPtr obstacleRad,
                                                             const DblVec& obstacleRadConfiguration)
    : m_env(primaryRad->GetEnv()),
      m_cc(CollisionChecker::GetOrCreate(*m_env)),
      m_rad(primaryRad),
      m_vars0(vars0),
      m_vars1(vars1),
      m_link2ind(),
      m_links(),
      m_obstacleRad(obstacleRad),
      m_obstacleConfiguration(obstacleRadConfiguration) {
  vector<int> inds;
  m_rad->GetAffectedLinks(m_links, true, inds);
  for (int i = 0; i < m_links.size(); ++i) {
    m_link2ind[m_links[i].get()] = inds[i];
  }
}

void CastDualArmCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);
  m_rad->SetDOFValues(dofvals0);
  m_obstacleRad->SetDOFValues(m_obstacleConfiguration);
  m_cc->CastVsAll(*m_rad, m_links, dofvals0, dofvals1, collisions);
}
void CastDualArmCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);
  CollisionsToDistanceExpressions(collisions, *m_rad, m_link2ind, m_vars0, m_vars1, dofvals0, dofvals1, exprs);
}
void CastDualArmCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, m_link2ind, dists);
}

void CastDualArmCollisionEvaluator::GetCollisionsCached(const DblVec& x, vector<Collision>& collisions) {
  DblVec concatenate(x);
  concatenate.insert(concatenate.end(), m_obstacleConfiguration.begin(), m_obstacleConfiguration.end());
  Base::GetCollisionsCached(concatenate, collisions);
}

//////////////////////////////////////////

typedef OpenRAVE::RaveVector<float> RaveVectorf;

void PlotCollisions(const std::vector<Collision>& collisions, OR::EnvironmentBase& env,
                    vector<OR::GraphHandlePtr>& handles, double safe_dist) {
  BOOST_FOREACH (const Collision& col, collisions) {
    RaveVectorf color;
    if (col.distance < 0)
      color = RaveVectorf(1, 0, 0, 1);
    else if (col.distance < safe_dist)
      color = RaveVectorf(1, 1, 0, 1);
    else
      color = RaveVectorf(0, 1, 0, 1);
    if (col.cctype == CCType_Between) {
      handles.push_back(env.drawarrow(col.ptB, col.ptB1, .002, RaveVectorf(0, 0, 0, 1)));
    }
    OR::Vector ptB = (col.cctype == CCType_Between) ? ((1 - col.time) * col.ptB + col.time * col.ptB1) : col.ptB;
    handles.push_back(env.drawarrow(col.ptA, ptB, .0025, color));
  }
}

CollisionCost::CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars)
    : Cost("collision"),
      m_calc(new SingleTimestepCollisionEvaluator(rad, vars)),
      m_dist_pen(dist_pen),
      m_coeff(coeff) {}

CollisionCost::CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars,
                             ConfigurationPtr obstacleRad, const DblVec& obstacleRadConfiguration)
    : Cost("dual_arm_collision"),
      m_calc(new SingleTimestepDualArmCollisionEvaluator(rad, vars, obstacleRad, obstacleRadConfiguration)),
      m_dist_pen(dist_pen),
      m_coeff(coeff) {}

CollisionCost::CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars0,
                             const VarVector& vars1)
    : Cost("cast_collision"),
      m_calc(new CastCollisionEvaluator(rad, vars0, vars1)),
      m_dist_pen(dist_pen),
      m_coeff(coeff) {}

CollisionCost::CollisionCost(double dist_pen, double coeff, ConfigurationPtr primaryRad, const VarVector& vars0,
                             const VarVector& vars1, ConfigurationPtr obstacleRad,
                             const DblVec& obstacleRadConfiguration)
    : Cost("cast_dual_arm_collision"),
      m_calc(new CastDualArmCollisionEvaluator(primaryRad, vars0, vars1, obstacleRad, obstacleRadConfiguration)),
      m_dist_pen(dist_pen),
      m_coeff(coeff) {}
ConvexObjectivePtr CollisionCost::convex(const vector<double>& x, Model* model) {
  ConvexObjectivePtr out(new ConvexObjective(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i = 0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addHinge(viol, m_coeff);
  }
  return out;
}
double CollisionCost::value(const vector<double>& x) {
  DblVec dists;
  m_calc->CalcDists(x, dists);
  double out = 0;
  for (int i = 0; i < dists.size(); ++i) {
    out += pospart(m_dist_pen - dists[i]) * m_coeff;
  }
  return out;
}

void CollisionCost::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  vector<Collision> collisions;
  m_calc->GetCollisionsCached(x, collisions);
  PlotCollisions(collisions, env, handles, m_dist_pen);
}

// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars)
    : m_calc(new SingleTimestepCollisionEvaluator(rad, vars)), m_dist_pen(dist_pen), m_coeff(coeff) {
  name_ = "collision";
}

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars0,
                                         const VarVector& vars1)
    : m_calc(new CastCollisionEvaluator(rad, vars0, vars1)), m_dist_pen(dist_pen), m_coeff(coeff) {
  name_ = "collision";
}
ConvexConstraintsPtr CollisionConstraint::convex(const vector<double>& x, Model* model) {
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i = 0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addIneqCnt(exprMult(viol, m_coeff));
  }
  return out;
}
DblVec CollisionConstraint::value(const vector<double>& x) {
  DblVec dists;
  m_calc->CalcDists(x, dists);
  DblVec out(dists.size());
  for (int i = 0; i < dists.size(); ++i) {
    out[i] = pospart(m_dist_pen - dists[i]) * m_coeff;
  }
  return out;
}

}  // namespace trajopt
