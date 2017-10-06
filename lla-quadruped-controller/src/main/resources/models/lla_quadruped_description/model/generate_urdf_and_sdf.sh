#!/bin/bash

# generate urdf model
rosrun xacro xacro robots/babyBeastSimple.xacro > urdf/babyBeast.urdf
# generate urdf model with head
sed '/<\/robot>/d' urdf/babyBeast.urdf > urdf/babyBeastWithHead.urdf
cat urdf/head.urdf >> urdf/babyBeastWithHead.urdf
tr -d '\r' < urdf/babyBeastWithHead.urdf > temp && mv temp urdf/babyBeastWithHead.urdf
echo '</robot>' >> urdf/babyBeastWithHead.urdf
# generate sdf models
gzsdf print urdf/babyBeast.urdf > sdf/babyBeast.sdf
gzsdf print urdf/babyBeastWithHead.urdf > sdf/babyBeastWithHead.sdf
