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

// RBDyn
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

/// @return An simple Z*12 arm with Y as up axis.
std::tuple<rbd::MultiBody, rbd::MultiBodyConfig> makeZ12Arm(bool isFixed=true)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  MultiBodyGraph mbg;

  double mass = 1.;
  Matrix3d I = Matrix3d::Identity();
  Vector3d h = Vector3d::Zero();

  RBInertiad rbi(mass, h, I);

  for(int i = 0; i < 13; ++i)
  {
    std::stringstream ss;
    ss << "b" << i;
    mbg.addBody({rbi, ss.str()});
  }

  for(int i = 0; i < 12; ++i)
  {
    std::stringstream ss;
    ss << "j" << i;
    mbg.addJoint({Joint::RevZ, true, ss.str()});
  }

  PTransformd to(Vector3d(0., 0.5, 0.));
  PTransformd from(Vector3d(0., 0., 0.));

  mbg.linkBodies("b0", PTransformd::Identity(), "b1", from, "j0");
  for(int i = 1; i < 12; ++i)
  {
    std::stringstream from_b_ss; from_b_ss << "b" << i;
    std::stringstream to_b_ss; to_b_ss << "b" << i+1;
    std::stringstream j_ss; j_ss << "j" << i;
    mbg.linkBodies(from_b_ss.str(), to, to_b_ss.str(), from, j_ss.str());
  }

  MultiBody mb = mbg.makeMultiBody("b0", isFixed);

  MultiBodyConfig mbc(mb);
  mbc.zero(mb);

  return std::make_tuple(mb, mbc);
}
