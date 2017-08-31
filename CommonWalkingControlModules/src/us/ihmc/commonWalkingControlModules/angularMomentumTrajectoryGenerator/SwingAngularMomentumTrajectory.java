package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SwingAngularMomentumTrajectory extends AngularMomentumTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.SWING;

   public SwingAngularMomentumTrajectory(int stepNumber, ReferenceFrame referenceFrame, int maxNumberOfSegments,
                                         int maxNumberOfCoefficients)
   {
      super(stepNumber, type, referenceFrame, maxNumberOfSegments, maxNumberOfCoefficients);
   }

}
