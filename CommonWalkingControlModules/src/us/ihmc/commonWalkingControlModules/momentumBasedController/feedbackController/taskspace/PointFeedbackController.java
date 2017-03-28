package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import us.ihmc.commonWalkingControlModules.controlModules.BodyFixedPointLinearAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class PointFeedbackController implements FeedbackControllerInterface
{
   private final YoVariableRegistry registry;

   private final BooleanYoVariable isEnabled;

   private final YoFramePoint yoDesiredPosition;
   private final YoFramePoint yoCurrentPosition;

   private final YoFrameVector yoDesiredLinearVelocity;
   private final YoFrameVector yoCurrentLinearVelocity;

   private final YoFrameVector yoFeedForwardLinearAcceleration;
   private final YoFrameVector yoDesiredLinearAcceleration;
   private final YoFrameVector yoAchievedLinearAcceleration;

   private final FrameVector achievedLinearAcceleration = new FrameVector();

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameVector desiredLinearAcceleration = new FrameVector();

   private final PointAccelerationCommand output = new PointAccelerationCommand();

   private final YoPositionPIDGainsInterface gains;
   private final BodyFixedPointLinearAccelerationControlModule accelerationControlModule;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private RigidBody base;

   private final RigidBody endEffector;

   public PointFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
         YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;
      spatialAccelerationCalculator = toolbox.getSpatialAccelerationCalculator();

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "PointFBController");
      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      double dt = toolbox.getControlDT();
      gains = feedbackControllerToolbox.getPositionGains(endEffector);
      accelerationControlModule = new BodyFixedPointLinearAccelerationControlModule(endEffectorName, twistCalculator, endEffector, dt, gains, registry);

      isEnabled = new BooleanYoVariable(endEffectorName + "isPointFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredPosition = feedbackControllerToolbox.getOrCreatePosition(endEffector, Type.DESIRED);
      yoCurrentPosition = feedbackControllerToolbox.getOrCreatePosition(endEffector, Type.CURRENT);

      yoDesiredLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.LINEAR_VELOCITY);
      yoCurrentLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.CURRENT, Space.LINEAR_VELOCITY);

      yoFeedForwardLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDFORWARD, Space.LINEAR_ACCELERATION);
      yoDesiredLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.LINEAR_ACCELERATION);
      yoAchievedLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.ACHIEVED, Space.LINEAR_ACCELERATION);

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(PointFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();

      output.set(command.getPointAccelerationCommand());

      gains.set(command.getGains());

      command.getBodyFixedPointIncludingFrame(tempPosition);
      accelerationControlModule.setPointToControl(tempPosition);

      command.getIncludingFrame(tempPosition, tempLinearVelocity, feedForwardLinearAcceleration);
      yoDesiredPosition.setAndMatchFrame(tempPosition);
      yoDesiredLinearVelocity.setAndMatchFrame(tempLinearVelocity);
      yoFeedForwardLinearAcceleration.setAndMatchFrame(feedForwardLinearAcceleration);
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   @Override
   public void initialize()
   {
      accelerationControlModule.reset();
   }

   @Override
   public void compute()
   {
      if (!isEnabled())
         return;

      yoDesiredPosition.getFrameTupleIncludingFrame(tempPosition);
      yoDesiredLinearVelocity.getFrameTupleIncludingFrame(tempLinearVelocity);
      yoFeedForwardLinearAcceleration.getFrameTupleIncludingFrame(feedForwardLinearAcceleration);

      accelerationControlModule.compute(desiredLinearAcceleration, tempPosition, tempLinearVelocity, feedForwardLinearAcceleration, base);

      updatePositionVisualization();

      accelerationControlModule.getBodyFixedPoint(tempPosition);

      output.setLinearAcceleration(desiredLinearAcceleration);
   }

   private void updatePositionVisualization()
   {
      yoDesiredLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);

      accelerationControlModule.getBodyFixedPoint(tempPosition);
      yoCurrentPosition.setAndMatchFrame(tempPosition);

      accelerationControlModule.getBodyFixedPointCurrentLinearVelocity(tempLinearVelocity);
      yoCurrentLinearVelocity.setAndMatchFrame(tempLinearVelocity);
   }

   @Override
   public void computeAchievedAcceleration()
   {
      accelerationControlModule.getBodyFixedPoint(tempPosition);
      spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(base, endEffector, tempPosition, achievedLinearAcceleration);
      yoAchievedLinearAcceleration.setAndMatchFrame(achievedLinearAcceleration);
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   @Override
   public PointAccelerationCommand getOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return output;
   }
}
