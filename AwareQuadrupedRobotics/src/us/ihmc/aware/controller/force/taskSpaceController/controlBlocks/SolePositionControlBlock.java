package us.ihmc.aware.controller.force.taskSpaceController.controlBlocks;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceControlBlock;
import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SolePositionControlBlock implements QuadrupedTaskSpaceControlBlock
{
   private final RobotQuadrant robotQuadrant;
   private final ReferenceFrame soleFrame;
   private final FramePoint solePositionSetpoint;
   private final FrameVector soleLinearVelocitySetpoint;
   private final FrameVector soleForceFeedforwardSetpoint;
   private final EuclideanPositionController solePositionController;

   public SolePositionControlBlock(RobotQuadrant robotQuadrant, ReferenceFrame soleFrame, double controlDT, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.soleFrame = soleFrame;
      solePositionSetpoint = new FramePoint();
      soleLinearVelocitySetpoint = new FrameVector();
      soleForceFeedforwardSetpoint = new FrameVector();
      solePositionController = new EuclideanPositionController(robotQuadrant.getCamelCaseNameForStartOfExpression() + "solePosition", soleFrame, controlDT, registry);
   }

   public void setSolePosition(FramePoint solePositionSetpoint)
   {
     this.solePositionSetpoint.setIncludingFrame(solePositionSetpoint);
   }

   public void setSoleLinearVelocity(FrameVector soleLinearVelocitySetpoint)
   {
      this.soleLinearVelocitySetpoint.setIncludingFrame(soleLinearVelocitySetpoint);
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
      soleLinearVelocitySetpoint.setToZero();
      solePositionController.reset();
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates inputEstimates, QuadrupedTaskSpaceCommands outputCommands)
   {
      FrameVector soleLinearVelocityEstimate = inputEstimates.getSoleLinearVelocity(robotQuadrant);
      FrameVector soleForceCommand = outputCommands.getSoleForce(robotQuadrant);

      // compute sole force
      soleForceCommand.setToZero(soleFrame);
      solePositionSetpoint.changeFrame(soleFrame);
      soleLinearVelocitySetpoint.changeFrame(soleFrame);
      soleLinearVelocityEstimate.changeFrame(soleFrame);
      soleForceFeedforwardSetpoint.setToZero(soleFrame);
      solePositionController.compute(soleForceCommand, solePositionSetpoint, soleLinearVelocitySetpoint, soleLinearVelocityEstimate, soleForceFeedforwardSetpoint);
   }
}