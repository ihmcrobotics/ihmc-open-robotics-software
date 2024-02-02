package us.ihmc.perception.zedDriver.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(
      value =
      @Platform(
            includepath = {"../", "/usr/include/hidapi/"},
            resourcepath = "../",
            linkpath = "../",
            include = {"include/zed_open_driver_external.h"},
            link = {"zed-driver"},
            preload = {"zed-driver", "jniZEDOpenDriver"}
      ),
      target = "us.ihmc.perception.zedDriver.ZEDOpenDriver"
)
public class ZEDDriverInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {
   }
}
