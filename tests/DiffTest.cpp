// This file is part of PG.
//
// PG is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PG is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with PG.  If not, see <http://www.gnu.org/licenses/>.

// include
// std
#include <tuple>

// boost
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE Diff test
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// roboptim
#include <roboptim/core/finite-difference-gradient.hh>

// sch
#include <sch/S_Object/S_Sphere.h>

// PG
#include "ConfigStruct.h"
#include "PGData.h"
#include "FixedContactConstr.h"
#include "PlanarSurfaceConstr.h"
#include "StaticStabilityConstr.h"
#include "PositiveForceConstr.h"
#include "FrictionConeConstr.h"
#include "CollisionConstr.h"
#include "StdCostFunc.h"
#include "RobotLinkConstr.h"
#include "CylindricalSurfaceConstr.h"
#include "CoMHalfSpaceConstr.h"

// Arm
#include "XYZ12Arm.h"

const Eigen::Vector3d gravity(0., 9.81, 0.);

template <typename T>
double checkGradient(
  const roboptim::GenericDifferentiableFunction<T>& function,
  const typename roboptim::GenericDifferentiableFunction<T>::vector_t& x,
  double epsilon=1e-8)
{
  typename roboptim::GenericFunctionTraits<T>::jacobian_t jac(
    std::get<0>(function.jacobianSize()),
    std::get<1>(function.jacobianSize()));
  typename roboptim::GenericFunctionTraits<T>::jacobian_t jacF(
    std::get<0>(function.jacobianSize()),
    std::get<1>(function.jacobianSize()));

  roboptim::GenericFiniteDifferenceGradient<T> fdfunction(function, epsilon);
  function.jacobian(jac, x);
  fdfunction.jacobian(jacF, x);

  return (jac - jacF).norm();
}

template <typename T>
double checkQGradient(
 const roboptim::GenericDifferentiableFunction<T>& function,
 const typename roboptim::GenericDifferentiableFunction<T>::vector_t& x,
    const pg::PGData& pgdata)
{
  auto rows = std::get<0>(function.jacobianSize());
  auto cols = std::get<1>(function.jacobianSize());
  typename roboptim::GenericFunctionTraits<T>::jacobian_t jac(rows, cols);
  typename roboptim::GenericFunctionTraits<T>::jacobian_t jacF(rows, cols);

  roboptim::GenericFiniteDifferenceGradient<T> fdfunction(function, 1e-5);
  function.jacobian(jac, x);
  fdfunction.jacobian(jacF, x);

  int forceVar = pgdata.nrForcePoints()*3;
  return (jac.block(0, 0, rows, cols - forceVar) -
          jacF.block(0, 0, rows, cols - forceVar)).norm();
}

template <typename T>
double checkForceGradient(
 const roboptim::GenericDifferentiableFunction<T>& function,
 const typename roboptim::GenericDifferentiableFunction<T>::vector_t& x,
    const pg::PGData& pgdata)
{
  auto rows = std::get<0>(function.jacobianSize());
  auto cols = std::get<1>(function.jacobianSize());
  Eigen::MatrixXd jac(rows, cols);
  Eigen::MatrixXd jacF(rows, cols);

  roboptim::GenericFiniteDifferenceGradient<T> fdfunction(function);
  function.jacobian(jac, x);
  fdfunction.jacobian(jacF, x);

  int deb = pgdata.forceParamsBegin();
  return (jac.block(0, deb, rows, cols - deb) -
          jacF.block(0, deb, rows, cols - deb)).norm();
}


BOOST_AUTO_TEST_CASE(FixedContactPosTest)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  Eigen::Vector3d target(2., 0., 0.);
  sva::PTransformd surface(sva::PTransformd::Identity());

  pg::FixedPositionContactConstr fpc(&pgdata, 12, target, surface);

  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(fpc, x), 1e-4);
  }
}

BOOST_AUTO_TEST_CASE(FixedContactOriTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  Matrix3d oriTarget(sva::RotZ(cst::pi<double>()));
  sva::PTransformd surface(sva::RotZ(-cst::pi<double>()/2.), Vector3d::Random());

  pg::FixedOrientationContactConstr foc(&pgdata, 3, oriTarget, surface);

  for(int i = 0; i < 100; ++i)
  {
    VectorXd x(VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(foc, x), 1e-4);
  }
}

BOOST_AUTO_TEST_CASE(PlanarPositionContactTest)
{
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  sva::PTransformd target(Eigen::Vector3d(0., 1., 0.));
  sva::PTransformd surface(sva::PTransformd::Identity());

  pg::PlanarPositionContactConstr ppp(&pgdata, 12, target, surface);

  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(ppp, x), 1e-4);
  }
}

BOOST_AUTO_TEST_CASE(PlanarOrientationContactTest)
{
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  Eigen::Matrix3d oriTarget(sva::RotZ(-cst::pi<double>()));
  sva::PTransformd surface(sva::PTransformd::Identity());

  pg::PlanarOrientationContactConstr pop(&pgdata, 12, oriTarget, surface, 1);

  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(pop, x), 1e-4);
  }
}

BOOST_AUTO_TEST_CASE(PlanarInclusionTest)
{
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  sva::PTransformd targetSurface(sva::RotZ(-cst::pi<double>()), Eigen::Vector3d(0., 1., 0.));
  sva::PTransformd bodySurface(sva::RotZ(-cst::pi<double>()/2.), Eigen::Vector3d(0., 1., 0.));
  std::vector<Eigen::Vector2d> targetPoints = {{1., 1.}, {-0., 1.}, {-0., -1.}, {1., -1.}};
  std::vector<Eigen::Vector2d> surfPoints = {{0.1, 0.1}, {-0.1, 0.1}, {-0.1, -0.1}, {0.1, -0.1}};

  pg::PlanarInclusionConstr pi(&pgdata, 12, targetSurface, targetPoints,
                               bodySurface, surfPoints);

  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(pi, x), 1e-4);
  }
}

BOOST_AUTO_TEST_CASE(StaticStabilityTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrForces = 8*3;
  int nrVar = mb.nrParams() + nrForces + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams() + qBegin);

  sva::PTransformd bodySurface(sva::RotZ(-cst::pi<double>()/2.), Eigen::Vector3d(0., 1., 0.));
  std::vector<Vector2d> surfPoints = {{0.1, 0.1}, {-0.1, 0.1}, {-0.1, -0.1}, {0.1, -0.1}};
  std::vector<sva::PTransformd> points(surfPoints.size());
  for(std::size_t i = 0; i < points.size(); ++i)
  {
    points[i] = sva::PTransformd(Vector3d(surfPoints[i][0], surfPoints[i][1], 0.))*bodySurface;
  }
  pgdata.forces({pg::ForceContact{12, points, 0.7}, pg::ForceContact{0, points, 0.7}});

  pg::StaticStabilityConstr ss(&pgdata);

  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(ss, x), 1e-4);
  }
}

BOOST_AUTO_TEST_CASE(PositiveForceTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrForces = 8*3;
  int nrVar = mb.nrParams() + nrForces + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams() + qBegin);

  sva::PTransformd bodySurface(sva::RotZ(-cst::pi<double>()/2.), Eigen::Vector3d(0., 1., 0.));
  std::vector<Vector2d> surfPoints = {{0.1, 0.1}, {-0.1, 0.1}, {-0.1, -0.1}, {0.1, -0.1}};
  std::vector<sva::PTransformd> points(surfPoints.size());
  for(std::size_t i = 0; i < points.size(); ++i)
  {
    points[i] = sva::PTransformd(Vector3d(surfPoints[i][0], surfPoints[i][1], 0.))*bodySurface;
  }
  pgdata.forces({pg::ForceContact{12, points, 0.7}, pg::ForceContact{0, points, 0.7}});

  pg::PositiveForceConstr ss(&pgdata);

  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(ss, x), 1e-4);
  }
}

BOOST_AUTO_TEST_CASE(FrictionConeTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrForces = 8*3;
  int nrVar = mb.nrParams() + nrForces + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams() + qBegin);

  sva::PTransformd bodySurface(sva::RotZ(-cst::pi<double>()/2.), Eigen::Vector3d(0., 1., 0.));
  std::vector<Vector2d> surfPoints = {{0.1, 0.1}, {-0.1, 0.1}, {-0.1, -0.1}, {0.1, -0.1}};
  std::vector<sva::PTransformd> points(surfPoints.size());
  for(std::size_t i = 0; i < points.size(); ++i)
  {
    points[i] = sva::PTransformd(Vector3d(surfPoints[i][0], surfPoints[i][1], 0.))*bodySurface;
  }
  pgdata.forces({pg::ForceContact{12, points, 0.7}, pg::ForceContact{0, points, 0.7}});

  pg::FrictionConeConstr fc(&pgdata);

  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(fc, x), 1e-4);
  }
}


BOOST_AUTO_TEST_CASE(EnvCollisionTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  sch::S_Sphere hullBody(0.5);
  sch::S_Sphere hullEnv(0.5);
  hullEnv.setTransformation(pg::tosch(sva::PTransformd::Identity()));
  pg::EnvCollision ec(12, &hullBody, sva::PTransformd::Identity(),
                      &hullEnv, 0.1);

  pg::EnvCollisionConstr ecc(&pgdata, {ec});

  for(int i = 0; i < 50; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize())*3.14);
    BOOST_CHECK_SMALL(checkGradient(ecc, x, 1e-8), 1e-3);
  }
}


BOOST_AUTO_TEST_CASE(SelfCollisionTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  sch::S_Sphere hullBody1(0.2);
  sch::S_Sphere hullBody2(0.2);
  pg::SelfCollision sc(12, &hullBody1, sva::PTransformd::Identity(),
                       6, &hullBody2, sva::PTransformd::Identity(),
                       0.1);

  pg::SelfCollisionConstr scc(&pgdata, {sc});

  for(int i = 0; i < 50; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize())*3.14);
    BOOST_CHECK_SMALL(checkGradient(scc, x, 1e-8), 1e-3);
  }
}


BOOST_AUTO_TEST_CASE(StdCostFunctionTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrForces = 4*3;
  int nrVar = mb.nrParams() + nrForces + qBegin;

  std::vector<pg::PGData> pgdatas;
  pgdatas.push_back({mb, gravity, nrVar, qBegin, mb.nrParams() + qBegin});

  Vector3d target(1.5, 0., 0.);
  Matrix3d oriTarget(sva::RotZ(-cst::pi<double>()));
  int id = 12;
  Matrix3d frame(sva::RotX(-cst::pi<double>()/2.));
  Matrix3d frameEnd(sva::RotX(cst::pi<double>()/2.));
  std::vector<pg::ForceContact> forceContacts =
    {{0 , {sva::PTransformd(frame, Vector3d(0.01, 0., 0.)),
           sva::PTransformd(frame, Vector3d(-0.01, 0., 0.))}, 1.},
     {id, {sva::PTransformd(frameEnd, Vector3d(0.01, 0., 0.)),
      sva::PTransformd(frameEnd, Vector3d(-0.01, 0., 0.))}, 1.}};
  pgdatas.back().forces(forceContacts);

  pg::RunConfig runConfig;
  pg::RobotConfig robotConfig;
  runConfig.targetQ = mbc.q;
  for(int i = 1; i < mb.nrJoints(); ++i)
  {
    runConfig.targetQ[i] = {0.33*i};
  }

  robotConfig.postureScale = 1.44;
  robotConfig.torqueScale = 0.;
  robotConfig.forceScale = 2.33;
  robotConfig.ellipseCostScale = 0.;
  robotConfig.bodyPosTargets = {{12, target, 3.45}};
  robotConfig.bodyOriTargets = {{12, oriTarget, 5.06}};
  robotConfig.forceContactsMin = {{12, 1.87}};
  robotConfig.torqueContactsMin = {{12, Vector3d::Random(),
                                    Vector3d::Random().normalized(), 2.92}};
  robotConfig.normalForceTargets = {{12, 2., 1.33}};
  robotConfig.tanForceMin = {{12, 1.33}};

  pg::StdCostFunc cost(pgdatas, {robotConfig}, {runConfig});

  // an error could appear
  // apparently sva::rotationError is not right when
  // the sum of rotation error diagonal is near (-)1.
  // see A mathematical introduction to robotic manipulation Prop 2.5 p29.
  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdatas.back().pbSize())*3.14);
    BOOST_CHECK_SMALL(checkGradient(cost, x, 1e-5), 1e-3);
  }
}


BOOST_AUTO_TEST_CASE(RobotLinkTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int qBegin2 = qBegin + mb.nrParams();
  int nrVar = mb.nrParams()*2 + qBegin;

  pg::PGData pgdata1(mb, gravity, nrVar, qBegin, mb.nrParams() + qBegin);
  pg::PGData pgdata2(mb, gravity, nrVar, qBegin2, mb.nrParams() + qBegin2);
  sva::PTransformd b1T(Quaterniond(Vector4d::Random().normalized()),
                       Vector3d::Random());
  sva::PTransformd b2T(Quaterniond(Vector4d::Random().normalized()),
                       Vector3d::Random());

  pg::RobotLinkConstr rlc(&pgdata1, &pgdata2, {{12, b1T, b2T}, {6, b1T, b2T}});

  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata1.pbSize())*3.14);
    BOOST_CHECK_SMALL(checkGradient(rlc, x), 1e-4);
  }
}


BOOST_AUTO_TEST_CASE(FreeGripperPositionTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  Eigen::Quaterniond qs(Eigen::Vector4d::Random());
  Eigen::Quaterniond qt(Eigen::Vector4d::Random());
  qs.normalize();
  qt.normalize();
  sva::PTransformd target(qt, Vector3d::Random());
  sva::PTransformd surface(qs, Vector3d::Random());

  pg::CylindricalPositionConstr fgpc(&pgdata, 12, target, surface);

  for(int i = 0; i < 100; ++i)
  {
    VectorXd x(VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(fgpc, x), 1e-4);
  }
}


BOOST_AUTO_TEST_CASE(FreeGripperNVecTest)
{
  using namespace Eigen;
  namespace cst = boost::math::constants;

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  Eigen::Quaterniond qs(Eigen::Vector4d::Random());
  Eigen::Quaterniond qt(Eigen::Vector4d::Random());
  qs.normalize();
  qt.normalize();
  sva::PTransformd target(qt, Vector3d::Random());
  sva::PTransformd surface(qs, Vector3d::Random());

  pg::CylindricalNVecConstr fgnvc(&pgdata, 12, target, surface);

  for(int i = 0; i < 100; ++i)
  {
    VectorXd x(VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(fgnvc, x), 1e-4);
  }
}


BOOST_AUTO_TEST_CASE(CoMHalfSpaceTest)
{
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  std::tie(mb, mbc) = makeXYZ12Arm();

  int qBegin = 5;
  int nrVar = mb.nrParams() + qBegin;

  pg::PGData pgdata(mb, gravity, nrVar, qBegin, mb.nrParams());

  Eigen::Vector3d O(2., 0., 0.);
  Eigen::Vector3d n(0., 4., 0.);

  pg::CoMHalfSpaceConstr chs(&pgdata, {O,O*2}, {n,n*2});

  for(int i = 0; i < 100; ++i)
  {
    Eigen::VectorXd x(Eigen::VectorXd::Random(pgdata.pbSize()));
    BOOST_CHECK_SMALL(checkGradient(chs, x), 1e-4);
  }
}
