package us.ihmc.perception.slamWrapper;

import org.bytedeco.javacpp.annotation.*;
import org.bytedeco.javacpp.tools.*;

@Properties(value =
@Platform(
      includepath = "../",
      resourcepath = "../",
      linkpath = "../",
      include = {"include/factor_graph_external.h"},
      link = {"slam-wrapper"}),
      target = "SlamWrapper")
public class SlamWrapperInfoMapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {

   }
}
