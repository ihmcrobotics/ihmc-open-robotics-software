package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TransferCoPTrajectory extends CoPTrajectory
{
   private final static CoPTrajectoryType type = CoPTrajectoryType.TRANSFER;
   public TransferCoPTrajectory(String namePrefix, int stepNumber, int maxNumberOfSegments, YoVariableRegistry registry)
   {
      super(namePrefix, stepNumber, maxNumberOfSegments, type, registry);
   }
}
