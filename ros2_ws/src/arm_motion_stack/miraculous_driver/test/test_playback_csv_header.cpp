#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "playback_csv_header.hpp"

using miraculous_driver::CsvHeaderFormat;
using miraculous_driver::classify_playback_csv_header;

TEST(PlaybackCsvHeaderTest, FiveJointV2TakesPrecedenceOverLegacyColumnCount)
{
  const std::vector<std::string> header{
    "timestamp", "sample_index", "J1", "J2", "J3", "J4", "J5"};

  EXPECT_EQ(classify_playback_csv_header(header, 6), CsvHeaderFormat::kV2);
}

TEST(PlaybackCsvHeaderTest, RecognizesSixJointLegacyHeader)
{
  const std::vector<std::string> header{
    "timestamp", "J1", "J2", "J3", "J4", "J5", "J6"};

  EXPECT_EQ(classify_playback_csv_header(header, 6), CsvHeaderFormat::kLegacy);
}

TEST(PlaybackCsvHeaderTest, RejectsUnknownHeader)
{
  const std::vector<std::string> header{"time", "sample", "J1"};

  EXPECT_EQ(classify_playback_csv_header(header, 6), CsvHeaderFormat::kUnsupported);
}
