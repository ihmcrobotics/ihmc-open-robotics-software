#!/bin/sh
set -x
mkdir -p doxygen/md
doxybook2 --input doxygen/xml --output doxygen/md
echo "[[_TOC_]]" > $1/api.md
cat doxygen/md/Classes/classpromp_1_1ProMP.md \
    doxygen/md/Classes/classpromp_1_1Trajectory.md \
    doxygen/md/Classes/classpromp_1_1TrajectoryGroup.md \
    doxygen/md/Classes/classpromp_1_1io_1_1CSVReader.md >> $1/api.md
sed -i.md s/"Classes\/classpromp_1_1ProMP.md"//g $1/api.md
sed -i.md s/"Classes\/classpromp_1_1Trajectory.md"//g $1/api.md
sed -i.md s/"Classes\/classpromp_1_1TrajectoryGroup.md"//g $1/api.md
sed -i.md s/"Classes\/classpromp_1_1io_1_1CSVReader.md"//g $1/api.md
