package us.ihmc.aware.controller.force.taskSpaceController.controlBlocks;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceControlBlock;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class BodyOrientationControlBlock implements QuadrupedTaskSpaceControlBlock
{
   private final ReferenceFrame bodyFrame;
   private final FrameOrientation bodyOrientationSetpoint;
   private final FrameVector bodyAngularVelocitySetpoint;
   private final FrameVector bodyTorqueFeedforwardSetpoint;
   private final AxisAngleOrientationController bodyOrientationController;

   public BodyOrientationControlBlock(ReferenceFrame bodyFrame, double controlDT, YoVariableRegistry registry)
   {
      this.bodyFrame = bodyFrame;
      bodyOrientationSetpoint = new FrameOrientation();
      bodyAngularVelocitySetpoint = new FrameVector();
      bodyTorqueFeedforwardSetpoint = new FrameVector();
      bodyOrientationController = new AxisAngleOrientationController("bodyOrientation", bodyFrame, controlDT, registry);
   }

   public void setBodyOrientation(FrameOrientation bodyOrientationSetpoint)
   {
      this.bodyOrientationSetpoint.setIncludingFrame(bodyOrientationSetpoint);
   }

   public void setBodyAngularVelocity(FrameVector bodyAngularVelocitySetpoint)
   {
      this.bodyAngularVelocitySetpoint.setIncludingFrame(bodyAngularVelocitySetpoint);
   }

   public void setProportionalGains(double[] proportionalGains)
   {
      bodyOrientationController.setProportionalGains(proportionalGains);
   }

   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      bodyOrientationController.setIntegralGains(integralGains, maxIntegralError);
   }

   public void setDerivativeGains(double[] derivativeGains)
   {
      bodyOrientationController.setDerivativeGains(derivativeGains);
   }

   @Override public void reset()
   {
      bodyAngularVelocitySetpoint.setToZero();
      bodyOrientationController.reset();
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates inputEstimates, QuadrupedTaskSpaceCommands outputCommands)
   {
      FrameVector bodyAngularVelocityEstimate = inputEstimates.getBodyAngularVelocity();
      FrameVector comTorqueCommand = outputCommands.getComTorque();

      // compute body torque
      comTorqueCommand.setToZero(bodyFrame);
      bodyOrientationSetpoint.changeFrame(bodyFrame);
      bodyAngularVelocitySetpoint.changeFrame(bodyFrame);
      bodyAngularVelocityEstimate.changeFrame(bodyFrame);
      bodyTorqueFeedforwardSetpoint.setToZero(bodyFrame);
      bodyOrientationController.compute(comTorqueCommand, bodyOrientationSetpoint, bodyAngularVelocitySetpoint, bodyAngularVelocityEstimate, bodyTorqueFeedforwardSetpoint);
   }
}