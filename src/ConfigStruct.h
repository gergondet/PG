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

#pragma once

// include
// std
#include <vector>

// boost
#include <boost/math/constants/constants.hpp>

// Eigen
#include <Eigen/Core>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include <RBDyn/MultiBody.h>


// forward declaration
namespace sch
{
class S_Object;
}

namespace pg
{

struct FixedPositionContact
{
  FixedPositionContact() {}
  FixedPositionContact(const std::string & bName, const Eigen::Vector3d& t,
                       const sva::PTransformd& sf)
    : bodyName(bName)
    , target(t)
    , surfaceFrame(sf)
  {}

  std::string bodyName;
  Eigen::Vector3d target; ///< Position target in world coordinate.
  sva::PTransformd surfaceFrame; ///< Body surface frame in body coordinate.
};


struct FixedOrientationContact
{
  FixedOrientationContact() {}
  FixedOrientationContact(const std::string & bName, const Eigen::Matrix3d& t,
                          const sva::PTransformd& sf)
    : bodyName(bName)
    , target(t)
    , surfaceFrame(sf)
  {}

  std::string bodyName;
  Eigen::Matrix3d target; ///< Orientation target in world coordinate.
  sva::PTransformd surfaceFrame; ///< Body surface frame in body coordinate.
};


struct PlanarContact
{
  PlanarContact() {}
  PlanarContact(const std::string & bName,
                const sva::PTransformd& tf, std::vector<Eigen::Vector2d> tp,
                const sva::PTransformd& sf, std::vector<Eigen::Vector2d> sp)
    : bodyName(bName)
    , targetFrame(tf)
    , targetPoints(std::move(tp))
    , surfaceFrame(sf)
    , surfacePoints(std::move(sp))
  {}

  std::string bodyName;
  sva::PTransformd targetFrame; ///< Target frame in world coordinate.
  std::vector<Eigen::Vector2d> targetPoints; ///< Target surface points in surface coordinate.
  sva::PTransformd surfaceFrame; ///< Body surface frame in body coordinate.
  std::vector<Eigen::Vector2d> surfacePoints; ///< Body surface points in surface coordinate.
};


struct EllipseContact
{
  EllipseContact() {}
  EllipseContact(const std::string & bName, double rMin,
                 const sva::PTransformd& tf, std::vector<Eigen::Vector2d> tp,
                 const sva::PTransformd& sf, std::vector<Eigen::Vector2d> sp)
    : bodyName(bName)
    , radiusMin1(rMin)
    , radiusMin2(rMin)
    , targetFrame(tf)
    , targetPoints(std::move(tp))
    , surfaceFrame(sf)
    , surfacePoints(std::move(sp))
  {
    assert( rMin > 0 && "rMin can't be negative");
  }
  EllipseContact(const std::string & bName, double rMin1, double rMin2,
                 const sva::PTransformd& tf, std::vector<Eigen::Vector2d> tp,
                 const sva::PTransformd& sf, std::vector<Eigen::Vector2d> sp)
    : bodyName(bName)
    , radiusMin1(rMin1)
    , radiusMin2(rMin2)
    , targetFrame(tf)
    , targetPoints(std::move(tp))
    , surfaceFrame(sf)
    , surfacePoints(std::move(sp))
  {
    assert( (rMin1 > 0 || rMin2 > 0) && "rMin1 and rMin2 can't be both negative");
    if (rMin2 < 0 && rMin1 >= 0)
      radiusMin2 = rMin1;
    else if (rMin1 < 0 && rMin2 >= 0)
      radiusMin1 = rMin2;
  }

  std::string bodyName;
  double radiusMin1;
  double radiusMin2;
  sva::PTransformd targetFrame; ///< Target frame in world coordinate.
  std::vector<Eigen::Vector2d> targetPoints; ///< Target surface points in surface coordinate.
  sva::PTransformd surfaceFrame; ///< Body surface frame in body coordinate.
  std::vector<Eigen::Vector2d> surfacePoints; ///< Body surface points in surface coordinate.
};


struct GripperContact
{
  GripperContact() {}
  GripperContact(const std::string & bName,
                const sva::PTransformd& tf, std::vector<Eigen::Vector2d> tp,
                const sva::PTransformd& sf, std::vector<Eigen::Vector2d> sp)
    : bodyName(bName)
    , targetFrame(tf)
    , targetPoints(std::move(tp))
    , surfaceFrame(sf)
    , surfacePoints(std::move(sp))
  {}

  std::string bodyName;
  sva::PTransformd targetFrame; ///< Target frame in world coordinate.
  std::vector<Eigen::Vector2d> targetPoints; ///< Target surface points in surface coordinate.
  sva::PTransformd surfaceFrame; ///< Body surface frame in body coordinate.
  std::vector<Eigen::Vector2d> surfacePoints; ///< Body surface points in surface coordinate.
};


struct CylindricalContact
{
  CylindricalContact() {}
  CylindricalContact(const std::string & bName, double tR, double tW,
                     const sva::PTransformd& tf, const sva::PTransformd& sf)
    : bodyName(bName)
    , targetRadius(tR)
    , targetWidth(tW)
    , targetFrame(tf)
    , surfaceFrame(sf)
  {}

  std::string bodyName;
  double targetRadius;
  double targetWidth;
  sva::PTransformd targetFrame; ///< Target frame in world coordinate.
  sva::PTransformd surfaceFrame; ///< Body surface frame in body coordinate.
};


struct ForceContact
{
  ForceContact() {}
//GD   ForceContact(const std::string & bName, std::vector<sva::PTransformd> p, double m)
  ForceContact(const std::string & bName, std::vector<sva::PTransformd> p, double m, double l=-1)
    : bodyName(bName)
    , points(std::move(p))
    , mu(m)
//GD<
    , limit(l)
//>GD
  {}

  std::string bodyName;
  std::vector<sva::PTransformd> points;
  double mu;
//GD<
  double limit;
//>GD  
};


struct EnvCollision
{
  EnvCollision() {}
  EnvCollision(const std::string & bName, sch::S_Object* bHull, const sva::PTransformd& bT,
               sch::S_Object* eHull,
               double md)
    : bodyName(bName)
    , bodyHull(bHull)
    , bodyT(bT)
    , envHull(eHull)
    , minDist(md)
  {}

  std::string bodyName;
  sch::S_Object* bodyHull;
  sva::PTransformd bodyT;
  sch::S_Object* envHull;
  double minDist;
};


struct SelfCollision
{
  SelfCollision() {}
  SelfCollision(const std::string & b1Name, sch::S_Object* b1Hull, const sva::PTransformd& b1T,
                const std::string & b2Name, sch::S_Object* b2Hull, const sva::PTransformd& b2T,
                double md)
    : body1Name(b1Name)
    , body1Hull(b1Hull)
    , body1T(b1T)
    , body2Name(b2Name)
    , body2Hull(b2Hull)
    , body2T(b2T)
    , minDist(md)
  {}

  std::string body1Name;
  sch::S_Object* body1Hull;
  sva::PTransformd body1T;
  std::string body2Name;
  sch::S_Object* body2Hull;
  sva::PTransformd body2T;
  double minDist;
};


struct CoMHalfSpace
{
  CoMHalfSpace() {}
  CoMHalfSpace(std::vector<Eigen::Vector3d> O,
               std::vector<Eigen::Vector3d> n)
    : origins(std::move(O))
    , normals(std::move(n))
  {}

  std::vector<Eigen::Vector3d> origins; ///< origin of the plane
  std::vector<Eigen::Vector3d> normals; ///< normal of the plane
};


struct BodyPositionTarget
{
  BodyPositionTarget() {}
  BodyPositionTarget(const std::string & bName, const Eigen::Vector3d& t, double s)
    : bodyName(bName)
    , target(t)
    , scale(s)
  {}

  std::string bodyName;
  Eigen::Vector3d target;
  double scale;
};


struct BodyOrientationTarget
{
  BodyOrientationTarget() {}
  BodyOrientationTarget(const std::string & bName, const Eigen::Matrix3d& t, double s)
    : bodyName(bName)
    , target(t)
    , scale(s)
  {}

  std::string bodyName;
  Eigen::Matrix3d target;
  double scale;
};


struct ForceContactMinimization
{
  ForceContactMinimization() {}
  ForceContactMinimization(const std::string & bName, double s)
    : bodyName(bName)
    , scale(s)
  {}

  std::string bodyName;
  double scale;
};


struct TorqueContactMinimization
{
  TorqueContactMinimization() {}
  TorqueContactMinimization(const std::string & bName, const Eigen::Vector3d& o,
                            const Eigen::Vector3d& a, double s)
    : bodyName(bName)
    , origin(o)
    , axis(a)
    , scale(s)
  {}

  std::string bodyName;
  Eigen::Vector3d origin, axis;
  double scale;
};


struct NormalForceTarget
{
  NormalForceTarget() {}
  NormalForceTarget(const std::string & bName, double t, double s)
    : bodyName(bName)
    , target(t)
    , scale(s)
  {}

  std::string bodyName;
  double target;
  double scale;
};


struct TangentialForceMinimization
{
  TangentialForceMinimization() {}
  TangentialForceMinimization(const std::string & bName, double s)
    : bodyName(bName)
    , scale(s)
  {}

  std::string bodyName;
  double scale;
};


struct EllipseResult
{
  int bodyIndex;  //Each ellipse is defined relatively to a Surface of a Body
  double x;     //x coord of the center
  double y;     //y coord of the center
  double theta; //Angle between the x-axis and the first axis of the ellipse
  double r1;    //First radius
  double r2;    //Second radius
  std::string print()
  {
    std::stringstream result;
    result << "ellipse = Ellipse((" << this->x << ", " << this->y << "), ";
    result << 2*this->r1 << ", " << 2*this->r2 << ", " << 180*this->theta/boost::math::constants::pi<double>() << ")\n";
    return result.str();
  }
};


struct RobotConfig
{
  RobotConfig()
    : postureScale(0.)
    , torqueScale(0.)
    , forceScale(0.)
    , ellipseCostScale(0.)
  {}

  RobotConfig(rbd::MultiBody multibody)
    : mb(std::move(multibody))
    , postureScale(0.)
    , torqueScale(0.)
    , forceScale(0.)
    , ellipseCostScale(0.)
  {}

  // robot
  rbd::MultiBody mb;

  // constraints
  std::vector<FixedPositionContact> fixedPosContacts;
  std::vector<FixedOrientationContact> fixedOriContacts;
  std::vector<PlanarContact> planarContacts;
  std::vector<EllipseContact> ellipseContacts;
  std::vector<GripperContact> gripperContacts;
  std::vector<CylindricalContact> cylindricalContacts;
  std::vector<ForceContact> forceContacts;
  std::vector<EnvCollision> envCollisions;
  std::vector<SelfCollision> selfCollisions;
  std::vector<CoMHalfSpace> comHalfSpaces;
  std::vector<std::vector<double>> ql, qu;
  std::vector<std::vector<double>> tl, tu;
  std::vector<std::vector<Eigen::VectorXd>> tlPoly, tuPoly;

  // costs
  double postureScale;
  double torqueScale;
  double forceScale;
  double ellipseCostScale;
  std::vector<BodyPositionTarget> bodyPosTargets;
  std::vector<BodyOrientationTarget> bodyOriTargets;
  std::vector<ForceContactMinimization> forceContactsMin;
  std::vector<TorqueContactMinimization> torqueContactsMin;
  std::vector<NormalForceTarget> normalForceTargets;
  std::vector<TangentialForceMinimization> tanForceMin;
};


struct BodyLink
{
  BodyLink()
    : bodyName("Root")
    , body1T(sva::PTransformd::Identity())
    , body2T(sva::PTransformd::Identity())
  {}

  BodyLink(const std::string & bName, const sva::PTransformd& b1T, const sva::PTransformd& b2T)
    : bodyName(bName)
    , body1T(b1T)
    , body2T(b2T)
  {}

  std::string bodyName;
  sva::PTransformd body1T;
  sva::PTransformd body2T;
};


struct RobotLink
{
  RobotLink()
    : robot1Index(-1)
    , robot2Index(-1)
    , linkedBodies()
  {}

  RobotLink(int r1Index, int r2Index, std::vector<BodyLink> lbodies)
    : robot1Index(r1Index)
    , robot2Index(r2Index)
    , linkedBodies(std::move(lbodies))
  {}

  int robot1Index;
  int robot2Index;
  std::vector<BodyLink> linkedBodies;
};


struct RunConfig
{
  RunConfig(){}
  RunConfig(std::vector<std::vector<double>> iQ,
            std::vector<sva::ForceVecd> iF,
            std::vector<std::vector<double>> tQ)
    : initQ(std::move(iQ))
    , initForces(std::move(iF))
    , targetQ(std::move(tQ))
  {}

  std::vector<std::vector<double>> initQ;
  std::vector<sva::ForceVecd> initForces;
  std::vector<std::vector<double>> targetQ;
};


struct IterateQuantities
{
  double obj, constr_viol;
};

} // namespace pg
