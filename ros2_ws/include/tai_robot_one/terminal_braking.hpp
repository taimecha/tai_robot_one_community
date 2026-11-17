// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__TERMINAL_BRAKING_HPP_
#define TAI_ROBOT_ONE__TERMINAL_BRAKING_HPP_
#include <algorithm>
#include <cmath>
namespace tai_robot_one
{
inline double angularStoppingTravel(double speed, double latency, double deceleration)
{
  if (!std::isfinite(speed) || !std::isfinite(latency) || !std::isfinite(deceleration) ||
    latency < 0.0 || deceleration <= 0.0)
  {
    return INFINITY;
  }
  return std::abs(speed) * latency + speed * speed / (2.0 * deceleration);
}

inline bool shouldBrakeTerminalTurn(
  double remaining, double speed, double latency, double deceleration)
{
  return std::isfinite(remaining) && std::isfinite(speed) &&
         remaining * speed > 0.0 && std::abs(speed) > 0.02 &&
         std::abs(remaining) <= angularStoppingTravel(speed, latency, deceleration);
}

inline double boundedLoadedTurnCommand(
  double remaining, double measured_rate, double limit, double minimum,
  double acceleration, double dt)
{
  if (!std::isfinite(remaining) || !std::isfinite(measured_rate) ||
    !std::isfinite(limit) || !std::isfinite(minimum) ||
    !std::isfinite(acceleration) || !std::isfinite(dt) ||
    limit <= 0.0 || minimum < 0.0 || acceleration <= 0.0 || dt <= 0.0)
  {
    return 0.0;
  }
  double command = std::copysign(std::min(limit,
      std::sqrt(2.0 * acceleration * std::abs(remaining))), remaining);
  command = std::clamp(command, measured_rate - acceleration * dt,
      measured_rate + acceleration * dt);
  // The acceleration clamp follows measured motion. If the measured rate
  // already exceeds the limit, it must not lift the requested rate above
  // that limit again. The drive retains its independent deceleration ramp.
  return std::copysign(std::min(limit, std::max(std::abs(command), minimum)), command);
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__TERMINAL_BRAKING_HPP_
