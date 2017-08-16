package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TransferAngularMomentumTrajectory extends AngularMomentumTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.TRANSFER;

   public TransferAngularMomentumTrajectory(String namePrefix, int stepNumber, YoVariableRegistry registry, ReferenceFrame referenceFrame,
                                            int maxNumberOfSegments, int maxNumberOfCoefficients)
   {
      super(namePrefix, stepNumber, type, registry, referenceFrame, maxNumberOfSegments, maxNumberOfCoefficients);
   }

}