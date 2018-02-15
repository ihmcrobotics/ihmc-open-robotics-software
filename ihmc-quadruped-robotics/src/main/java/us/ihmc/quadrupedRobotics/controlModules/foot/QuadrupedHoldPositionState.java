package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedHoldPositionState extends QuadrupedFootState
{
   // YoVariables
   private final YoVariableRegistry registry;
   private final YoDouble timestamp;
   private double initialTime;

   private final ParameterFactory parameterFactory;
   private final BooleanParameter useSoleForceFeedForwardParameter;
   private final DoubleParameter feedForwardRampTimeParameter;

   // SoleWaypoint variables
   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();

   // Feedback controller
   private final QuadrupedSolePositionControllerSetpoints solePositionControllerSetpoints;
   private final FrameVector3D initialSoleForces = new FrameVector3D();
   private final QuadrupedSolePositionController solePositionController;

   private final ReferenceFrame bodyFrame;
   private final QuadrupedFootControlModuleParameters parameters;
   private final RobotQuadrant robotQuadrant;

   public QuadrupedHoldPositionState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox toolbox, QuadrupedSolePositionController solePositionController,
                                     YoVariableRegistry parentRegistry)
   {
      this.robotQuadrant = robotQuadrant;
      this.solePositionController = solePositionController;

      bodyFrame = toolbox.getReferenceFrames().getBodyFrame();
      parameters = toolbox.getFootControlModuleParameters();
      timestamp = toolbox.getRuntimeEnvironment().getRobotTimestamp();

      registry = new YoVariableRegistry(robotQuadrant.getShortName() + getClass().getSimpleName());

      parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
      useSoleForceFeedForwardParameter = parameterFactory.createBoolean("useSoleForceFeedForward", true);
      feedForwardRampTimeParameter = parameterFactory.createDouble("feedForwardRampTime", 2.0);

      solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);

      parentRegistry.addChild(registry);
   }

   @Override
   public void updateEstimates(QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      this.taskSpaceEstimates.set(taskSpaceEstimates);
   }

   @Override
   public void onEntry()
   {
      initialTime = timestamp.getDoubleValue();

      solePositionController.reset();
      solePositionController.getGains().setProportionalGains(parameters.getSolePositionProportionalGainsParameter());
      solePositionController.getGains().setDerivativeGains(parameters.getSolePositionDerivativeGainsParameter());
      solePositionController.getGains().setIntegralGains(parameters.getSolePositionIntegralGainsParameter(), parameters.getSolePositionMaxIntegralErrorParameter());

      solePositionControllerSetpoints.initialize(taskSpaceEstimates);
      FramePoint3D solePositionSetpoint = solePositionControllerSetpoints.getSolePosition();
      solePositionSetpoint.setIncludingFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
      solePositionSetpoint.changeFrame(bodyFrame);

      FrameVector3DReadOnly forceEstimate = taskSpaceEstimates.getSoleVirtualForce(robotQuadrant);
      initialSoleForces.setIncludingFrame(forceEstimate);
      initialSoleForces.changeFrame(bodyFrame);
   }

   @Override
   public QuadrupedFootControlModule.FootEvent process()
   {
      solePositionControllerSetpoints.getSoleLinearVelocity().setToZero();
      solePositionControllerSetpoints.getSoleForceFeedforward().setIncludingFrame(initialSoleForces);
      solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, taskSpaceEstimates);

      double currentTime = timestamp.getDoubleValue();
      if (useSoleForceFeedForwardParameter.get())
      {
         double rampMultiplier = 1.0 - Math.min(1.0, (currentTime - initialTime) / feedForwardRampTimeParameter.get());
         FrameVector3D feedforward = solePositionControllerSetpoints.getSoleForceFeedforward();
         feedforward.set(initialSoleForces);
         feedforward.scale(rampMultiplier);
      }
      solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, taskSpaceEstimates);

      return null;
   }

   @Override
   public void onExit()
   {
      soleForceCommand.setToZero();
   }
}
