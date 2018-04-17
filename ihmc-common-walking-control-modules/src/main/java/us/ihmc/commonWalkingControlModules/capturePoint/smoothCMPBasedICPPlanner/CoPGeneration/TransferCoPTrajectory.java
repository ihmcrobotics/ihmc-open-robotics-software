package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;

public class TransferCoPTrajectory extends CoPTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.TRANSFER;
   public TransferCoPTrajectory(CoPSplineType splineType, int maxNumberOfSegments)
   {
      super(splineType, maxNumberOfSegments, type);
   }  
}
