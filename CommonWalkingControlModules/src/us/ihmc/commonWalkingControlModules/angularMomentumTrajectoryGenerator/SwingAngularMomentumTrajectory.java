package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SwingAngularMomentumTrajectory extends AngularMomentumTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.SWING;
   
   public SwingAngularMomentumTrajectory(String namePrefix, int stepNumber, YoVariableRegistry registry, ReferenceFrame referenceFrame, int maxNumberOfWaypoints)
   {
      super(namePrefix, stepNumber, type, registry, referenceFrame, maxNumberOfWaypoints);
   }
   
}
