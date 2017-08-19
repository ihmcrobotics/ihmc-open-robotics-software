package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TransferCoPTrajectory extends CoPTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.TRANSFER;
   public TransferCoPTrajectory(int stepNumber, CoPSplineType splineType, int maxNumberOfSegments)
   {
      super(stepNumber, splineType, maxNumberOfSegments, type);
   }  
}
