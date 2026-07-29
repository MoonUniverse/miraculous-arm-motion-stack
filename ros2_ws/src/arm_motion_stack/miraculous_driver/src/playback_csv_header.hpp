#ifndef MIRACULOUS_DRIVER__PLAYBACK_CSV_HEADER_HPP_
#define MIRACULOUS_DRIVER__PLAYBACK_CSV_HEADER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace miraculous_driver
{

enum class CsvHeaderFormat
{
  kUnsupported,
  kLegacy,
  kV2,
};

inline CsvHeaderFormat classify_playback_csv_header(
  const std::vector<std::string> & header, size_t arm_joint_count)
{
  // V2 with five joint columns has the same total column count as six-axis
  // legacy CSV, so the explicit sample_index marker must take precedence.
  if (header.size() >= 3 &&
    header[0] == "timestamp" && header[1] == "sample_index")
  {
    return CsvHeaderFormat::kV2;
  }
  if (header.size() == arm_joint_count + 1 && header[0] == "timestamp") {
    return CsvHeaderFormat::kLegacy;
  }
  return CsvHeaderFormat::kUnsupported;
}

}  // namespace miraculous_driver

#endif  // MIRACULOUS_DRIVER__PLAYBACK_CSV_HEADER_HPP_
