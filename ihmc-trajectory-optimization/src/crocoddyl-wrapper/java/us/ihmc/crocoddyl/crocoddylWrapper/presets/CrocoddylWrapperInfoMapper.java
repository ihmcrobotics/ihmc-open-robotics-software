package us.ihmc.crocoddyl.crocoddylWrapper.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(
      value =
      @Platform(
            includepath = {"../"},
            resourcepath = "../",
            linkpath = "../",
            include = {"include/crocoddyl_external.h"},
            link = {"crocoddyl-wrapper"},
            preload = {"crocoddyl-wrapper", "jniCrocoddylWrapper"}
      ),
      target = "us.ihmc.crocoddyl.crocoddylWrapper.CrocoddylWrapper"
)
public class CrocoddylWrapperInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {
   }
}
