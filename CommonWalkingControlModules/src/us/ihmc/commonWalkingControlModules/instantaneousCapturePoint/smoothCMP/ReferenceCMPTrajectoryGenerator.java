package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class ReferenceCMPTrajectoryGenerator
{
   private final List<YoDouble> swingDurations;
   private final List<YoDouble> swingFractionsOnHeel;

   private final List<YoDouble> transferDurations;
   private final List<YoDouble> transferFractionsOnHeel;

   private final YoBoolean useSegmentedSwing;

   public ReferenceCMPTrajectoryGenerator(String namePrefix, List<YoDouble> swingDurations, List<YoDouble> transferDurations, List<YoDouble> swingFractionsOnHeel,
                                          List<YoDouble> transferFractionsOnHeel, YoBoolean useSegmentedSwing, YoVariableRegistry registry)
   {
      this.swingDurations = swingDurations;
      this.transferDurations = transferDurations;
      this.swingFractionsOnHeel = swingFractionsOnHeel;
      this.transferFractionsOnHeel = transferFractionsOnHeel;
      this.useSegmentedSwing = useSegmentedSwing;

   }


}
