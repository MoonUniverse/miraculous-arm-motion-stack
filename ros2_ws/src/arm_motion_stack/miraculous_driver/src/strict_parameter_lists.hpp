#ifndef MIRACULOUS_DRIVER__STRICT_PARAMETER_LISTS_HPP_
#define MIRACULOUS_DRIVER__STRICT_PARAMETER_LISTS_HPP_

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace miraculous_driver
{

inline std::string trim_parameter_token(const std::string & input)
{
  const auto first = std::find_if_not(
    input.begin(), input.end(),
    [](unsigned char character) {return std::isspace(character) != 0;});
  const auto last = std::find_if_not(
    input.rbegin(), input.rend(),
    [](unsigned char character) {return std::isspace(character) != 0;}).base();
  return first < last ? std::string(first, last) : std::string();
}

inline std::vector<std::string> split_parameter_list_strict(
  const std::string & input, const std::string & parameter_name)
{
  if (input.empty()) {
    return {};
  }
  if (input.front() == ',' || input.back() == ',') {
    throw std::invalid_argument(
            parameter_name + " contains an empty list element");
  }

  std::vector<std::string> tokens;
  size_t begin = 0;
  while (begin <= input.size()) {
    const size_t separator = input.find(',', begin);
    const size_t end = separator == std::string::npos ? input.size() : separator;
    const std::string token = trim_parameter_token(input.substr(begin, end - begin));
    if (token.empty()) {
      throw std::invalid_argument(
              parameter_name + " contains an empty list element");
    }
    tokens.push_back(token);
    if (separator == std::string::npos) {
      break;
    }
    begin = separator + 1;
  }
  return tokens;
}

inline std::vector<int> parse_int_parameter_list(
  const std::string & input, const std::vector<int> & default_value,
  const std::string & parameter_name)
{
  if (input.empty()) {
    return default_value;
  }

  std::vector<int> values;
  const auto tokens = split_parameter_list_strict(input, parameter_name);
  values.reserve(tokens.size());
  for (const auto & token : tokens) {
    size_t consumed = 0;
    int value = 0;
    try {
      value = std::stoi(token, &consumed);
    } catch (const std::exception &) {
      throw std::invalid_argument(
              parameter_name + " contains a non-integer value: " + token);
    }
    if (consumed != token.size()) {
      throw std::invalid_argument(
              parameter_name + " contains trailing characters: " + token);
    }
    values.push_back(value);
  }
  return values;
}

inline std::vector<double> parse_double_parameter_list(
  const std::string & input, const std::vector<double> & default_value,
  const std::string & parameter_name)
{
  if (input.empty()) {
    return default_value;
  }

  std::vector<double> values;
  const auto tokens = split_parameter_list_strict(input, parameter_name);
  values.reserve(tokens.size());
  for (const auto & token : tokens) {
    size_t consumed = 0;
    double value = 0.0;
    try {
      value = std::stod(token, &consumed);
    } catch (const std::exception &) {
      throw std::invalid_argument(
              parameter_name + " contains a non-numeric value: " + token);
    }
    if (consumed != token.size() || !std::isfinite(value)) {
      throw std::invalid_argument(
              parameter_name + " contains an invalid finite number: " + token);
    }
    values.push_back(value);
  }
  return values;
}

}  // namespace miraculous_driver

#endif  // MIRACULOUS_DRIVER__STRICT_PARAMETER_LISTS_HPP_
