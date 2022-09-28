package org.bytedeco.slamWrapper.presets;

import org.bytedeco.javacpp.annotation.*;
import org.bytedeco.javacpp.tools.*;

@Properties(
      value =
         @Platform(
            includepath = "../",
            resourcepath = "../",
            linkpath = "../",
            include = {"include/FactorGraphExternal.h"},
            link = {"slam-wrapper"}
         ),
      target = "org.bytedeco.slamWrapper.SlamWrapper"
)
public class SlamWrapperInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {
   }
}
