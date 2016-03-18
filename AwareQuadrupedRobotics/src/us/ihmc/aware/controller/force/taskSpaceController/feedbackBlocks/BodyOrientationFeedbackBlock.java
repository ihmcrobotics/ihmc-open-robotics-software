package us.ihmc.aware.controller.force.taskSpaceController.feedbackBlocks;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceFeedbackBlock;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class BodyOrientationFeedbackBlock implements QuadrupedTaskSpaceFeedbackBlock
{
   private final ReferenceFrame bodyFrame;
   private final FrameOrientation bodyOrientationSetpoint;
   private final FrameVector bodyAngularVelocitySetpoint;
   private final FrameVector bodyTorqueFeedforwardSetpoint;
   private final FrameVector bodyTorqueCommand;
   private final AxisAngleOrientationController bodyOrientationController;

   public BodyOrientationFeedbackBlock(ReferenceFrame bodyFrame, double controlDT, YoVariableRegistry registry)
   {
      this.bodyFrame = bodyFrame;
      bodyOrientationSetpoint = new FrameOrientation();
      bodyAngularVelocitySetpoint = new FrameVector();
      bodyTorqueFeedforwardSetpoint = new FrameVector();
      bodyTorqueCommand = new FrameVector();
      bodyOrientationController = new AxisAngleOrientationController("bodyOrientation", bodyFrame, controlDT, registry);
   }

   public void setBodyOrientationSetpoint(FrameOrientation bodyOrientationSetpoint)
   {
      this.bodyOrientationSetpoint.setIncludingFrame(bodyOrientationSetpoint);
   }

   public void setBodyAngularVelocitySetpoint(FrameVector bodyAngularVelocitySetpoint)
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
      bodyOrientationController.reset();
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceCommands commands)
   {
      FrameVector bodyAngularVelocityEstimate = estimates.getBodyAngularVelocity();
      bodyOrientationSetpoint.changeFrame(bodyFrame);
      bodyAngularVelocitySetpoint.setToZero(bodyFrame);
      bodyAngularVelocityEstimate.changeFrame(bodyFrame);
      bodyTorqueFeedforwardSetpoint.setToZero(bodyFrame);
      bodyTorqueCommand.changeFrame(bodyFrame);
      bodyOrientationController.compute(bodyTorqueCommand, bodyOrientationSetpoint, bodyAngularVelocitySetpoint, bodyAngularVelocityEstimate, bodyTorqueFeedforwardSetpoint);
      commands.getComTorque().setIncludingFrame(bodyTorqueCommand);
   }
}