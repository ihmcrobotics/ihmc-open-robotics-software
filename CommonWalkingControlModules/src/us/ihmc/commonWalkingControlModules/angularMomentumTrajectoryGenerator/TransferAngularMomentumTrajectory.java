package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TransferAngularMomentumTrajectory extends AngularMomentumTrajectory
{
   private final static WalkingTrajectoryType type = WalkingTrajectoryType.TRANSFER;
   
   public TransferAngularMomentumTrajectory(String namePrefix, int stepNumber, YoVariableRegistry registry, ReferenceFrame referenceFrame, int maxNumberOfWaypoints)
   {
      super(namePrefix, stepNumber, type, registry, referenceFrame, maxNumberOfWaypoints);
   }

}