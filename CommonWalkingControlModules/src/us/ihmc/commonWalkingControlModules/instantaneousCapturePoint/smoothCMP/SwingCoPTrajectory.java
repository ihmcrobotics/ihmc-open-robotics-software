package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SwingCoPTrajectory extends CoPTrajectory
{
   private final static CoPTrajectoryType type = CoPTrajectoryType.SWING;   
   public SwingCoPTrajectory(String namePrefix, int stepNumber, CoPSplineType splineType, int maxNumberOfSegments, YoVariableRegistry registry)
   {
      super(namePrefix, stepNumber, splineType, maxNumberOfSegments, type, registry);
   }

}
