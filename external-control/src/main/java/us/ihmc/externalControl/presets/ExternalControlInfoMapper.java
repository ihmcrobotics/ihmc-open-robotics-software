package us.ihmc.externalControl.presets;

import org.bytedeco.javacpp.annotation.Platform;
import org.bytedeco.javacpp.annotation.Properties;
import org.bytedeco.javacpp.tools.Info;
import org.bytedeco.javacpp.tools.InfoMap;
import org.bytedeco.javacpp.tools.InfoMapper;

// @formatter:off
@Properties(value =
      @Platform(value = "linux",
                includepath = "../include",
                include = {"external-control.hpp"},
                linkpath = "../lib",
                link = "external-wrapper",
                preload = {"external-wrapper", "jniExternalWrapper"}
      ),
      target = "us.ihmc.externalControl.ExternalControlWrapper"
)
// @formatter:on

public class ExternalControlInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {

      // Types

      // Renaming, example: infoMap.put(new Info("ihmc::Frame").pointerTypes("FrameImpl"));

      // Enums

      // Abstract
      infoMap.put(new Info("ihmc::ExternalControlImpl").virtualize());

      // Skipping some parts of the headers
      String skipStartRegex = "^ *// JAVACPP START SKIP *$";
      String skipEndRegex = "^ *// JAVACPP END SKIP *$";
      infoMap.put(new Info("external-control.hpp").linePatterns(skipStartRegex, skipEndRegex).skip());
   }
}
