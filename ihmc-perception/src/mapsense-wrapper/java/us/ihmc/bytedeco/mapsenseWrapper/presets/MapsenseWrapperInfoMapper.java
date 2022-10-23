package us.ihmc.bytedeco.mapsenseWrapper.presets;

import org.bytedeco.javacpp.annotation.*;
import org.bytedeco.javacpp.tools.*;

@Properties(
      value =
      @Platform(
            includepath = "../",
            resourcepath = "../",
            linkpath = "../",
            include = {"include/mapsense_external.h"},
            link = {"mapsense-wrapper"}
      ),
      target = "us.ihmc.bytedeco.mapsenseWrapper.MapsenseWrapper"
)
public class MapsenseWrapperInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {
   }
}
