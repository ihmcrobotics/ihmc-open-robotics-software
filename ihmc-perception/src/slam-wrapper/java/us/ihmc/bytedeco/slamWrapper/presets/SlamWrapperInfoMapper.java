package us.ihmc.bytedeco.slamWrapper.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(
      value =
         @Platform(
            includepath = "../",
            resourcepath = "../",
            linkpath = "../",
            include = {"include/FactorGraphExternal.h"},
            link = {"slam-wrapper"},
            preload = {
//                  "tbb",
                  "boost_filesystem",
                  "boost_chrono",
                  "boost_timer",
                  "boost_serialization",
                  "metis-gtsam",
                  "gtsam",
                  "slam-wrapper",
                  "jniSlamWrapper"
            }
         ),
      target = "us.ihmc.bytedeco.slamWrapper.SlamWrapper"
)
public class SlamWrapperInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {
   }
}
