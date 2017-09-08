package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;

public class TransferCoPTrajectory extends CoPTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.TRANSFER;
   public TransferCoPTrajectory(int stepNumber, CoPSplineType splineType, int maxNumberOfSegments)
   {
      super(stepNumber, splineType, maxNumberOfSegments, type);
   }  
}
