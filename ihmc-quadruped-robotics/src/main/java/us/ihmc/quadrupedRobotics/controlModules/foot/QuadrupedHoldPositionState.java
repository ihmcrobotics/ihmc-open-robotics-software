package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedHoldPositionState extends QuadrupedUnconstrainedFootState
{
   // YoVariables
   private final YoVariableRegistry registry;
   private final YoDouble timestamp;
   private double initialTime;

   private final BooleanParameter useSoleForceFeedForwardParameter;
   private final DoubleParameter feedForwardRampTimeParameter;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame soleFrame;
   private final QuadrupedFootControlModuleParameters parameters;

   private final FrameVector3D soleLinearVelocityEstimate;

   public QuadrupedHoldPositionState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox controllerToolbox, QuadrupedSolePositionController solePositionController,
                                     YoVariableRegistry parentRegistry)
   {
      super(robotQuadrant, controllerToolbox, solePositionController);

      bodyFrame = controllerToolbox.getReferenceFrames().getBodyFrame();
      soleFrame = controllerToolbox.getSoleReferenceFrame(robotQuadrant);
      parameters = controllerToolbox.getFootControlModuleParameters();
      timestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      soleLinearVelocityEstimate = controllerToolbox.getTaskSpaceEstimates().getSoleLinearVelocity(robotQuadrant);

      registry = new YoVariableRegistry(robotQuadrant.getPascalCaseName() + getClass().getSimpleName());

      useSoleForceFeedForwardParameter = new BooleanParameter("useSoleForceFeedForward", registry, true);
      feedForwardRampTimeParameter = new DoubleParameter("feedForwardRampTime", registry, 2.0);

      parentRegistry.addChild(registry);
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
   public QuadrupedFootControlModule.FootEvent process()
   {
      solePositionControllerSetpoints.getSoleLinearVelocity().setToZero();
      solePositionControllerSetpoints.getSoleForceFeedforward().setIncludingFrame(initialSoleForces);
      solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, soleLinearVelocityEstimate);

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

      return null;
   }

   @Override
   public void onExit()
   {
      soleForceCommand.setToZero();
   }
}
