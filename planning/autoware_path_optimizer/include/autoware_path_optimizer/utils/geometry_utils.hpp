// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_PATH_OPTIMIZER__UTILS__GEOMETRY_UTILS_HPP_
#define AUTOWARE_PATH_OPTIMIZER__UTILS__GEOMETRY_UTILS_HPP_

#include "autoware_path_optimizer/common_structs.hpp"
#include "autoware_path_optimizer/type_alias.hpp"
#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <Eigen/Core>

#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include "boost/optional/optional_fwd.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace tier4_autoware_utils
{
template <>
geometry_msgs::msg::Point getPoint(const autoware_path_optimizer::ReferencePoint & p);

template <>
geometry_msgs::msg::Pose getPose(const autoware_path_optimizer::ReferencePoint & p);
}  // namespace tier4_autoware_utils

namespace autoware_path_optimizer
{
namespace geometry_utils
{
template <typename T1, typename T2>
bool isSamePoint(const T1 & t1, const T2 & t2)
{
  const auto p1 = tier4_autoware_utils::getPoint(t1);
  const auto p2 = tier4_autoware_utils::getPoint(t2);

  constexpr double epsilon = 1e-6;
  if (epsilon < std::abs(p1.x - p2.x) || epsilon < std::abs(p1.y - p2.y)) {
    return false;
  }
  return true;
}

bool isOutsideDrivableAreaFromRectangleFootprint(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const vehicle_info_util::VehicleInfo & vehicle_info,
  const bool use_footprint_polygon_for_outside_drivable_area_check);
}  // namespace geometry_utils
}  // namespace autoware_path_optimizer
#endif  // AUTOWARE_PATH_OPTIMIZER__UTILS__GEOMETRY_UTILS_HPP_
