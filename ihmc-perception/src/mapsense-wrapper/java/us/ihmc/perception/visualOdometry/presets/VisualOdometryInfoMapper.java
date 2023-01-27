package us.ihmc.perception.visualOdometry.presets;

import org.bytedeco.javacpp.annotation.*;
import org.bytedeco.javacpp.tools.*;

@Properties(
      value =
      @Platform(
            includepath = {"../","/usr/include/opencv4","/usr/include/eigen3"},
            resourcepath = "../",
            linkpath = "../",
            include = {"include/visual_odometry_external.h"},
            link = {"visual-odometry"}
      ),
      target = "us.ihmc.perception.visualOdometry.VisualOdometry"
)
public class VisualOdometryInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {
   }
}
