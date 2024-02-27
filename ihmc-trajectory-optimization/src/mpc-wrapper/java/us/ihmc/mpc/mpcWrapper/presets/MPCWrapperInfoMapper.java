package us.ihmc.mpc.mpcWrapper.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

@Properties(
      value =
      @Platform(
            includepath = {"../", "/usr/local/include/eigen3"},
            resourcepath = "../",
            linkpath = "../",
            include = {"include/MPCExternal.h"},
            link = {"mpc-wrapper"},
            preload = {"mpc-wrapper", "jniMPCWrapper"}
      ),
      target = "us.ihmc.mpc.mpcWrapper.MPCWrapper"
)
public class MPCWrapperInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {
   }
}
