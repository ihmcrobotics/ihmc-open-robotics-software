package us.ihmc.aware.controller.force.taskSpaceController.feedbackBlocks;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceFeedbackBlock;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ComHeightFeedbackBlock implements QuadrupedTaskSpaceFeedbackBlock
{
   private double mass;
   private double gravity;
   private double controlDT;
   private double comHeightSetpoint;
   private double feedforwardConstant;
   private final PIDController comHeightController;

   public ComHeightFeedbackBlock(double controlDT, double mass, double gravity, YoVariableRegistry registry)
   {
      this.mass = mass;
      this.gravity = gravity;
      this.controlDT = controlDT;
      comHeightSetpoint = 1.0;
      feedforwardConstant = 1.0;
      comHeightController = new PIDController("comHeight", registry);
   }

   public void setComHeightSetpoint(double comHeightSetpoint)
   {
      this.comHeightSetpoint = comHeightSetpoint;
   }

   public void setProportionalGain(double proportionalGain)
   {
      comHeightController.setProportionalGain(proportionalGain);
   }

   public void setIntegralGain(double integralGain, double maxIntegralError)
   {
      comHeightController.setIntegralGain(integralGain);
      comHeightController.setMaxIntegralError(maxIntegralError);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      comHeightController.setDerivativeGain(derivativeGain);
   }

   public void setFeedforwardConstant(double feedforwardConstant)
   {
      this.feedforwardConstant = feedforwardConstant;
   }

   @Override public void reset() {
      comHeightController.resetIntegrator();
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceCommands commands)
   {
      estimates.getComVelocity().changeFrame(ReferenceFrame.getWorldFrame());
      double comHeightEstimate = estimates.getComHeight();
      double comHeightVelocityEstimate = estimates.getComVelocity().getZ();
      double comVerticalForceCommand = 0.0;
      comVerticalForceCommand += feedforwardConstant * mass * gravity;
      comVerticalForceCommand += comHeightController.compute(comHeightEstimate, comHeightSetpoint, comHeightVelocityEstimate, 0, controlDT);
      commands.getComForce().setZ(comVerticalForceCommand);
   }
}
