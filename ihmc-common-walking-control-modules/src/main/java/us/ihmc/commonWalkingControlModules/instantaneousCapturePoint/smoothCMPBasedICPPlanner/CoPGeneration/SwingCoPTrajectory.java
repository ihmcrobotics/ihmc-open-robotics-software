package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMPBasedICPPlanner.WalkingTrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SwingCoPTrajectory extends CoPTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.SWING;   
   public SwingCoPTrajectory(CoPSplineType splineType, int maxNumberOfSegments)
   {
      super(splineType, maxNumberOfSegments, type);
   }

}
