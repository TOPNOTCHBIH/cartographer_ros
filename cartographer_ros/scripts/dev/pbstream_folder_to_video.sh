#!/bin/bash

# Copyright 2018 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -o errexit
set -o verbose

DIR="$1"

if [ -z "$DIR" ] || [ -nd ${DIR} ]; then
  echo "Usage: $0 <folder with pbstream files> <arguments for cartographer_dev_pbstream_to_png_canvas>"
  exit 1;
fi

for FILENAME in "$DIR"/*.pbstream; do
  echo "Reading ${FILENAME}"
  rosrun cartographer_ros cartographer_dev_pbstream_to_png_canvas \
    --pbstream_filename="${FILENAME}" "${@:2}"
  convert "${FILENAME}.png" -level 10%,60% "${FILENAME}.png"
done

(cd $DIR &&
ffmpeg -pattern_type glob -i "*.pbstream.png" -r 30 \
    -c:v libx264 -preset slow -crf 5 -y output.mp4)
