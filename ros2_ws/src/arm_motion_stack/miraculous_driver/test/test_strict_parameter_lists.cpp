#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "strict_parameter_lists.hpp"

namespace miraculous_driver
{
namespace
{

TEST(StrictParameterListsTest, ParsesCompleteListsAndWhitespace)
{
  EXPECT_EQ(
    parse_int_parameter_list("1, 2,3", {}, "node_ids"),
    (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(
    parse_double_parameter_list("-1.0, 0.5", {}, "position_min"),
    (std::vector<double>{-1.0, 0.5}));
}

TEST(StrictParameterListsTest, UsesDefaultOnlyForAnActuallyEmptyParameter)
{
  EXPECT_EQ(
    parse_int_parameter_list("", {0, 1}, "joint_indices"),
    (std::vector<int>{0, 1}));
}

TEST(StrictParameterListsTest, RejectsMalformedOrPartialLists)
{
  EXPECT_THROW(
    parse_int_parameter_list("1,bad,3", {}, "node_ids"),
    std::invalid_argument);
  EXPECT_THROW(
    parse_int_parameter_list("1,2x,3", {}, "node_ids"),
    std::invalid_argument);
  EXPECT_THROW(
    parse_int_parameter_list("1,,3", {}, "node_ids"),
    std::invalid_argument);
  EXPECT_THROW(
    parse_double_parameter_list("-1,nan", {}, "position_min"),
    std::invalid_argument);
}

}  // namespace
}  // namespace miraculous_driver
