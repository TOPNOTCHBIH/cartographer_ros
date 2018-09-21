/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <map>
#include <string>

#include "cartographer/io/file_writer.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/3d/submap_3d.h"
#include "cartographer_ros/ros_map.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");
DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the drawn map [meters/pixel].");
DEFINE_int32(image_translate_x, 0, "Translate image by x [pixels].");
DEFINE_int32(image_translate_y, 0, "Translate image by y [pixels].");
DEFINE_double(image_width, 1920, "Image width [pixels].");
DEFINE_double(image_height, 1080, "Image height [pixels].");

namespace cartographer_ros {
namespace {

void Run(const std::string& pbstream_filename, const double resolution,
         const int image_translate_x, const int image_translate_y,
         const double image_width, const double image_height) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::ValueConversionTables conversion_tables;
  ::cartographer::io::DeserializeAndFillSubmapSlices(
      &deserializer, &submap_slices, &conversion_tables);
  CHECK(reader.eof());

  LOG(INFO) << "Generating combined map image from submap slices.";
  auto result =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
  ::cartographer::io::Image image_slice(std::move(result.surface));
  LOG(INFO) << "If you want the output png to match the upper left corner of "
               "the rendered pbstream, set "
            << " --image_translate_x=" << static_cast<int>(-result.origin.x())
            << " --image_translate_y=" << static_cast<int>(-result.origin.y());
  LOG(INFO) << "Also, choose at least "
            << " --image_width=" << image_slice.width()
            << " --image_height=" << image_slice.height();

  ::cartographer::io::Image image_canvas(image_width, image_height);
  const ::cartographer::io::Uint8Color background_color{{128, 128, 128}};
  for (int canvas_y = 0; canvas_y < image_canvas.height(); ++canvas_y) {
    for (int canvas_x = 0; canvas_x < image_canvas.width(); ++canvas_x) {
      int x_in_image_slice = canvas_x + result.origin.x() + image_translate_x;
      int y_in_image_slice = canvas_y + result.origin.y() + image_translate_y;
      if (x_in_image_slice < 0 || x_in_image_slice >= image_slice.width() ||
          y_in_image_slice < 0 || y_in_image_slice >= image_slice.height()) {
        image_canvas.SetPixel(canvas_x, canvas_y, background_color);
        continue;
      }
      const uint16_t intensity =
          image_slice.GetPixel(x_in_image_slice, y_in_image_slice)[0];
      const uint16_t alpha =
          image_slice.GetPixel(x_in_image_slice, y_in_image_slice)[1];
      ::cartographer::io::Uint8Color color;
      for (int i = 0; i < 3; ++i) {
        color[i] =
            (intensity * alpha + (background_color[i] * (255 - alpha))) / 256;
      }
      image_canvas.SetPixel(canvas_x, canvas_y, color);
    }
  }
  ::cartographer::io::StreamFileWriter png_writer(pbstream_filename + ".png");
  image_canvas.WritePng(&png_writer);
  LOG(INFO) << "Wrote " << png_writer.GetFilename();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_pbstream_filename.empty()) << "-pbstream_filename is missing.";

  ::cartographer_ros::Run(FLAGS_pbstream_filename, FLAGS_resolution,
                          FLAGS_image_translate_x, FLAGS_image_translate_y,
                          FLAGS_image_width, FLAGS_image_height);
}
