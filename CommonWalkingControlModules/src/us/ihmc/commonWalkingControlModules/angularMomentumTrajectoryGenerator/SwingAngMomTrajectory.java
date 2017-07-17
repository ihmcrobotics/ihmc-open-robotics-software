package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SwingAngMomTrajectory extends AngularMomentumTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.SWING;

   public SwingAngMomTrajectory(String namePrefix, int stepNumber, YoVariableRegistry registry, ReferenceFrame referenceFrame, int maxNumberOfSegments,
                                         int maxNumberOfCoefficients)
   {
      super(namePrefix, stepNumber, type, registry, referenceFrame, maxNumberOfSegments, maxNumberOfCoefficients);
   }

}
