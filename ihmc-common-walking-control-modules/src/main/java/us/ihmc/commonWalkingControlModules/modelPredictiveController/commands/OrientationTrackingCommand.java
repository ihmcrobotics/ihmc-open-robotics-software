package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoMTrajectoryModelPredictiveController;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCIndexHandler;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class OrientationTrackingCommand implements MPCCommand<OrientationTrackingCommand>
{
   private final CubicTrackingCommand yawTrackingCommand = new CubicTrackingCommand();
   private final CubicTrackingCommand pitchTrackingCommand = new CubicTrackingCommand();
   private final CubicTrackingCommand rollTrackingCommand = new CubicTrackingCommand();

   private double weight = CoMTrajectoryModelPredictiveController.orientationTrackingWeight;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.ORIENTATION_TRACKING;
   }

   public void setStartOrientation(FrameQuaternionReadOnly initialOrientation)
   {
      yawTrackingCommand.setStartValue(initialOrientation.getYaw());
      pitchTrackingCommand.setStartValue(initialOrientation.getPitch());
      rollTrackingCommand.setStartValue(initialOrientation.getRoll());
   }

   public void setFinalOrientation(FrameQuaternionReadOnly finalOrientation)
   {
      yawTrackingCommand.setFinalValue(finalOrientation.getYaw());
      pitchTrackingCommand.setFinalValue(finalOrientation.getPitch());
      rollTrackingCommand.setFinalValue(finalOrientation.getRoll());
   }

   public void setStartAngularRate(FrameVector3DReadOnly initialAngularRate)
   {
      yawTrackingCommand.setStartRate(initialAngularRate.getX());
      pitchTrackingCommand.setStartRate(initialAngularRate.getY());
      rollTrackingCommand.setStartRate(initialAngularRate.getZ());
   }

   public void setFinalAngularRate(FrameVector3DReadOnly finalAngularRate)
   {
      yawTrackingCommand.setFinalRate(finalAngularRate.getX());
      pitchTrackingCommand.setFinalRate(finalAngularRate.getY());
      rollTrackingCommand.setFinalRate(finalAngularRate.getZ());
   }

   public void setSegmentNumber(int segmentNumber, MPCIndexHandler indexHandler)
   {
      yawTrackingCommand.setStartIndex(indexHandler.getYawCoefficientsStartIndex(segmentNumber));
      pitchTrackingCommand.setStartIndex(indexHandler.getPitchCoefficientsStartIndex(segmentNumber));
      rollTrackingCommand.setStartIndex(indexHandler.getRollCoefficientsStartIndex(segmentNumber));
   }

   public void setSegmentDuration(double duration)
   {
      yawTrackingCommand.setDuration(duration);
      pitchTrackingCommand.setDuration(duration);
      rollTrackingCommand.setDuration(duration);
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public CubicTrackingCommand getYawTrackingCommand()
   {
      return yawTrackingCommand;
   }

   public CubicTrackingCommand getPitchTrackingCommand()
   {
      return pitchTrackingCommand;
   }

   public CubicTrackingCommand getRollTrackingCommand()
   {
      return rollTrackingCommand;
   }

   public double getWeight()
   {
      return weight;
   }
}
