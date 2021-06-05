////////////////////////////////////////////////////////////////////////////////
// Copyright 2021 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_trajectory_segment.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/01/20
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_trajectory_interpolation/cartesian_trajectory_segment.h>
#include <algorithm>
#include <cmath>
#include "Eigen/src/Core/GlobalFunctions.h"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include <Eigen/Dense>

namespace cartesian_ros_control
{
  using Time = CartesianTrajectorySegment::Time;
  using SplineState = CartesianTrajectorySegment::SplineState;

  CartesianTrajectorySegment::CartesianTrajectorySegment(const Time&  start_time,
      const CartesianState& start_state,
      const Time&  end_time,
      const CartesianState& end_state)
    : QuinticSplineSegment(start_time, convert(start_state), end_time, convert(end_state))
  {
  };


  void CartesianTrajectorySegment::sample(const Time& time, CartesianState& state) const
  {
    // Sample from the underlying spline segment.
    SplineState s(7);
    QuinticSplineSegment::sample(time, s);

    if (time < this->startTime() || time > this->endTime())
    {
      state.p = Eigen::Vector3d(s.position[0], s.position[1], s.position[2]);
      state.q = Eigen::Quaterniond(s.position[3], s.position[4], s.position[5], s.position[6]).normalized();

      state.v = Eigen::Vector3d::Zero();
      state.w = Eigen::Vector3d::Zero();
      state.v_dot = Eigen::Vector3d::Zero();
      state.w_dot = Eigen::Vector3d::Zero();
    }
    else
    {
      state = convert(s);
    }
  }


  SplineState convert(const CartesianState& state)
  {
    SplineState spline_state;

    // Note: The pre-multiplication of velocity and acceleration terms with
    // `state.q.inverse()` transforms them into the body-local reference frame.
    // This is required for computing quaternion-based velocities and
    // accelerations below.

    // Convenience method
    auto fill = [](auto& vec, const auto& first, const auto& second)
    {
      vec.push_back(first.x());
      vec.push_back(first.y());
      vec.push_back(first.z());

      vec.push_back(second.w());
      vec.push_back(second.x());
      vec.push_back(second.y());
      vec.push_back(second.z());
    };

    // Spline positions
    fill(spline_state.position, state.p, state.q);

    // Obsolete after Eigen 3.4, which provides begin() and end() for Eigen vectors.
    auto std = [](Eigen::Vector3d v){return std::vector<double>(v.data(), v.data() + v.size());};

    // Check all vector fields for NaNs.
    // Any occurence indicates a deliberately uninitialzed value.
    auto uninitialized = [std](Eigen::Vector3d v)->bool{
      return std::any_of(std(v).begin(), std(v).end(), [](double d){return std::isnan(d);});
    };

    // Spline velocities
    if (uninitialized(state.v) || uninitialized(state.w))
    {
      return spline_state;  // with uninitialized velocity/acceleration data
    }
    Eigen::Quaterniond q_dot;
    Eigen::Vector3d tmp = state.q.inverse() * state.w;
    Eigen::Quaterniond omega(0, tmp.x(), tmp.y(), tmp.z());
    q_dot.coeffs() = 0.5 * (omega * state.q).coeffs();

    fill(spline_state.velocity, state.q.inverse() * state.v, q_dot);

    // Spline accelerations
    if (uninitialized(state.v_dot) || uninitialized(state.w_dot))
    {
      return spline_state;  // with uninitialized acceleration data
    }
    Eigen::Quaterniond q_ddot;
    tmp = state.q.inverse() * state.w_dot;
    Eigen::Quaterniond omega_dot(0, tmp.x(), tmp.y(), tmp.z());
    q_ddot.coeffs() = 0.5 * (omega_dot * state.q).coeffs() + 0.5 * (omega * q_dot).coeffs();

    fill(spline_state.acceleration, state.q.inverse() * state.v_dot, q_ddot);

    return spline_state;
  };


  CartesianState convert(const SplineState& s)
  {
    CartesianState state;

    // Cartesian positions
    state.p = Eigen::Vector3d(s.position[0], s.position[1], s.position[2]);
    state.q = Eigen::Quaterniond(s.position[3], s.position[4], s.position[5], s.position[6]).normalized();

    // Cartesian velocities
    if (s.velocity.empty())
    {
      return state;  // with velocities/accelerations zero initialized
    }
    Eigen::Quaterniond q_dot(s.velocity[3], s.velocity[4], s.velocity[5], s.velocity[6]);

    Eigen::Quaterniond omega;
    omega.coeffs() = 2.0 * (q_dot * state.q.inverse()).coeffs();

    state.v = Eigen::Vector3d(s.velocity[0], s.velocity[1], s.velocity[2]);
    state.w = Eigen::Vector3d(omega.x(), omega.y(), omega.z());

    // Cartesian accelerations
    if (s.acceleration.empty())
    {
      return state;  // with accelerations zero initialized
    }
    Eigen::Quaterniond q_ddot(s.acceleration[3], s.acceleration[4], s.acceleration[5], s.acceleration[6]);

    Eigen::Quaterniond omega_dot;
    omega_dot.coeffs() = 2.0 * (
        (q_ddot * state.q.inverse()).coeffs() -
        ((q_dot * state.q.inverse()) * (q_dot * state.q.inverse())).coeffs()
        );

    state.v_dot = Eigen::Vector3d(s.acceleration[0], s.acceleration[1], s.acceleration[2]);
    state.w_dot = Eigen::Vector3d(omega_dot.x(), omega_dot.y(), omega_dot.z());

    // Re-transform vel and acc to the correct reference frame.
    state.v = state.q * state.v;
    state.w = state.q * state.w;
    state.v_dot = state.q * state.v_dot;
    state.w_dot = state.q * state.w_dot;

    return state;
  }

  std::ostream& operator<<(std::ostream &out, const CartesianTrajectorySegment::SplineState& state)
  {
    out << "pos:\n";
    for (size_t i = 0; i < state.position.size(); ++i)
    {
      out << state.position[i] << '\n';
    }
    out << "vel:\n";
    for (size_t i = 0; i < state.velocity.size(); ++i)
    {
      out << state.velocity[i] << '\n';
    }
    out << "acc:\n";
    for (size_t i = 0; i < state.acceleration.size(); ++i)
    {
      out << state.acceleration[i] << '\n';
    }

    return out;
  }
}
