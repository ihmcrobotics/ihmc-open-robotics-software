package us.ihmc.perception.slamWrapper;

import org.bytedeco.javacpp.annotation.*;
import org.bytedeco.javacpp.tools.*;

@Properties(value =
@Platform(
      include = {"include/factor_graph_external.h"},
      link = {"libihmc-slam-wrapper"}),
      target = "FactorGraphExternal")
public class ihmc_slam_wrapper implements InfoMapper
{
   public void map(InfoMap infoMap)
   {

   }
}
