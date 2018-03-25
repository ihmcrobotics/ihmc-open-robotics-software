package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedHoldPositionState extends QuadrupedUnconstrainedFootState
{
   // YoVariables
   private final YoDouble timestamp;
   private double initialTime;

   private final BooleanParameter useSoleForceFeedForwardParameter;
   private final DoubleParameter feedForwardRampTimeParameter;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame soleFrame;
   private final QuadrupedFootControlModuleParameters parameters;

   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionControllerSetpoints solePositionControllerSetpoints;

   private final PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();

   private final FrameVector3D soleLinearVelocityEstimate;

   public QuadrupedHoldPositionState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox controllerToolbox,
                                     QuadrupedSolePositionController solePositionController, YoVariableRegistry registry)
   {
      super(robotQuadrant, controllerToolbox);

      this.solePositionController = solePositionController;
      solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);

      bodyFrame = controllerToolbox.getReferenceFrames().getBodyFrame();
      soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);
      parameters = controllerToolbox.getFootControlModuleParameters();
      timestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      soleLinearVelocityEstimate = controllerToolbox.getTaskSpaceEstimates().getSoleLinearVelocity(robotQuadrant);

      RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotQuadrant);
      FramePoint3D currentPosition = new FramePoint3D(soleFrame);
      currentPosition.changeFrame(foot.getBodyFixedFrame());

      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getBody(), foot);
      feedbackControlCommand.setBodyFixedPointToControl(currentPosition);

      useSoleForceFeedForwardParameter = new BooleanParameter("useSoleForceFeedForward", registry, true);
      feedForwardRampTimeParameter = new DoubleParameter("feedForwardRampTime", registry, 2.0);
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      initialTime = timestamp.getDoubleValue();

      solePositionController.reset();
      solePositionController.getGains().set(parameters.getSolePositionGains());

      solePositionControllerSetpoints.initialize(soleFrame);
      FramePoint3D solePositionSetpoint = solePositionControllerSetpoints.getSolePosition();
      solePositionSetpoint.setToZero(soleFrame);
      solePositionSetpoint.changeFrame(bodyFrame);

      FrameVector3DReadOnly forceEstimate = controllerToolbox.getTaskSpaceEstimates().getSoleVirtualForce(robotQuadrant);
      initialSoleForces.setIncludingFrame(forceEstimate);
      initialSoleForces.changeFrame(bodyFrame);
   }

   @Override
   public void doAction(double timeInState)
   {
      solePositionControllerSetpoints.getSoleLinearVelocity().setToZero();
      solePositionControllerSetpoints.getSoleForceFeedforward().setIncludingFrame(initialSoleForces);
      solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, soleLinearVelocityEstimate);

      feedbackControlCommand.set(solePositionControllerSetpoints.getSolePosition(), solePositionControllerSetpoints.getSoleLinearVelocity());
      feedbackControlCommand.setFeedForwardAction(initialSoleForces);
      feedbackControlCommand.setGains(parameters.getSolePositionGains());

      virtualForceCommand.setLinearForce(soleFrame, soleForceCommand);

      double currentTime = timestamp.getDoubleValue();
      if (useSoleForceFeedForwardParameter.getValue())
      {
         double rampMultiplier = 1.0 - Math.min(1.0, (currentTime - initialTime) / feedForwardRampTimeParameter.getValue());
         FrameVector3D feedforward = solePositionControllerSetpoints.getSoleForceFeedforward();
         feedforward.set(initialSoleForces);
         feedforward.scale(rampMultiplier);
      }
      solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, soleLinearVelocityEstimate);

      super.doControl();
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return null;
   }

   @Override
   public void onExit()
   {
      soleForceCommand.setToZero();
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }
}
