package us.ihmc.aware.controller.force.taskSpaceController.feedbackBlocks;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceFeedbackBlock;
import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SolePositionFeedbackBlock implements QuadrupedTaskSpaceFeedbackBlock
{
   private final RobotQuadrant robotQuadrant;
   private final ReferenceFrame soleFrame;
   private final FramePoint solePositionSetpoint;
   private final FrameVector soleLinearVelocitySetpoint;
   private final FrameVector soleForceFeedforwardSetpoint;
   private final FrameVector soleForceCommand;
   private final EuclideanPositionController solePositionController;

   public SolePositionFeedbackBlock(RobotQuadrant robotQuadrant, ReferenceFrame soleFrame, double controlDT, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.soleFrame = soleFrame;
      solePositionSetpoint = new FramePoint();
      soleLinearVelocitySetpoint = new FrameVector();
      soleForceFeedforwardSetpoint = new FrameVector();
      soleForceCommand = new FrameVector();
      String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression() + "solePosition";
      solePositionController = new EuclideanPositionController(prefix, soleFrame, controlDT, registry);
   }

   public void setSolePositionSetpoint(FramePoint solePositionSetpoint)
   {
     this.solePositionSetpoint.setIncludingFrame(solePositionSetpoint);
   }

   public void setSoleLinearVelocitySetpoint(FrameVector soleLinearVelocitySetpoint)
   {
      this.soleLinearVelocitySetpoint.setIncludingFrame(soleLinearVelocitySetpoint);
   }

   public void setSoleForceFeedforwardSetpoint(FrameVector soleForceFeedforwardSetpoint)
   {
      this.soleForceFeedforwardSetpoint.setIncludingFrame(soleForceFeedforwardSetpoint);
   }

   public void setProportionalGains(double[] proportionalGains)
   {
      solePositionController.setProportionalGains(proportionalGains);
   }

   public void setIntegralGainsGains(double[] integralGains, double maxIntegralError)
   {
      solePositionController.setIntegralGains(integralGains, maxIntegralError);
   }

   public void setDerivativeGains(double[] derivativeGains)
   {
      solePositionController.setDerivativeGains(derivativeGains);
   }

   @Override public void reset()
   {
      solePositionController.reset();
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceCommands commands)
   {
      FrameVector soleLinearVelocityEstimate = estimates.getSoleLinearVelocity().get(robotQuadrant);
      soleForceCommand.setToZero();
      soleForceCommand.changeFrame(soleFrame);
      solePositionSetpoint.changeFrame(soleFrame);
      soleLinearVelocitySetpoint.changeFrame(soleFrame);
      soleLinearVelocityEstimate.changeFrame(soleFrame);
      soleForceFeedforwardSetpoint.changeFrame(soleFrame);
      solePositionController.compute(soleForceCommand, solePositionSetpoint, soleLinearVelocitySetpoint, soleLinearVelocityEstimate, soleForceFeedforwardSetpoint);
      commands.getSoleForce().get(robotQuadrant).setIncludingFrame(soleForceCommand);
   }
}