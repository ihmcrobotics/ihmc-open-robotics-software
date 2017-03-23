package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import us.ihmc.commonWalkingControlModules.controlModules.BodyFixedPointSpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class SpatialFeedbackController implements FeedbackControllerInterface
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

   private final YoFrameQuaternion yoDesiredOrientation;
   private final YoFrameQuaternion yoCurrentOrientation;

   private final YoFrameVector yoDesiredRotationVector;
   private final YoFrameVector yoCurrentRotationVector;

   private final YoFrameVector yoDesiredAngularVelocity;
   private final YoFrameVector yoCurrentAngularVelocity;

   private final YoFrameVector yoFeedForwardAngularAcceleration;
   private final YoFrameVector yoDesiredAngularAcceleration;
   private final YoFrameVector yoAchievedAngularAcceleration;

   private final SpatialAccelerationVector endEffectorAchievedAcceleration = new SpatialAccelerationVector();
   private final FrameVector achievedAngularAcceleration = new FrameVector();
   private final FrameVector achievedLinearAcceleration = new FrameVector();

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameVector desiredLinearAcceleration = new FrameVector();

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final FrameVector desiredAngularAcceleration = new FrameVector();

   private final SpatialAccelerationVector desiredSpatialAcceleration = new SpatialAccelerationVector();

   private final AxisAngle tempAxisAngle = new AxisAngle();

   private final SpatialAccelerationCommand output = new SpatialAccelerationCommand();

   private final BodyFixedPointSpatialAccelerationControlModule accelerationControlModule;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private RigidBody base;

   private final RigidBody endEffector;
   private final ReferenceFrame endEffectorFrame;

   public SpatialFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
         YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;
      spatialAccelerationCalculator = toolbox.getSpatialAccelerationCalculator();

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "SpatialFBController");
      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      double dt = toolbox.getControlDT();
      accelerationControlModule = new BodyFixedPointSpatialAccelerationControlModule(endEffectorName, twistCalculator, endEffector, dt, registry);

      endEffectorFrame = endEffector.getBodyFixedFrame();

      isEnabled = new BooleanYoVariable(endEffectorName + "isSpatialFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredPosition = feedbackControllerToolbox.getOrCreatePosition(endEffector, Type.DESIRED);
      yoCurrentPosition = feedbackControllerToolbox.getOrCreatePosition(endEffector, Type.CURRENT);

      yoDesiredLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.LINEAR_VELOCITY);
      yoCurrentLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.CURRENT, Space.LINEAR_VELOCITY);

      yoFeedForwardLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDFORWARD, Space.LINEAR_ACCELERATION);
      yoDesiredLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.LINEAR_ACCELERATION);
      yoAchievedLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.ACHIEVED, Space.LINEAR_ACCELERATION);

      yoDesiredOrientation = feedbackControllerToolbox.getOrCreateOrientation(endEffector, Type.DESIRED);
      yoCurrentOrientation = feedbackControllerToolbox.getOrCreateOrientation(endEffector, Type.CURRENT);

      yoDesiredRotationVector = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.ROTATION_VECTOR);
      yoCurrentRotationVector = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.CURRENT, Space.ROTATION_VECTOR);

      yoDesiredAngularVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.ANGULAR_VELOCITY);
      yoCurrentAngularVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.CURRENT, Space.ANGULAR_VELOCITY);

      yoFeedForwardAngularAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDFORWARD, Space.ANGULAR_ACCELERATION);
      yoDesiredAngularAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.ANGULAR_ACCELERATION);
      yoAchievedAngularAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.ACHIEVED, Space.ANGULAR_ACCELERATION);

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(SpatialFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();
      output.set(command.getSpatialAccelerationCommand());

      accelerationControlModule.setGains(command.getGains());

      command.getControlFramePoseIncludingFrame(tempPosition, tempOrientation);
      accelerationControlModule.setBodyFixedControlFrame(tempPosition, tempOrientation);

      command.getIncludingFrame(tempPosition, tempLinearVelocity, feedForwardLinearAcceleration);
      yoDesiredPosition.setAndMatchFrame(tempPosition);
      yoDesiredLinearVelocity.setAndMatchFrame(tempLinearVelocity);
      yoFeedForwardLinearAcceleration.setAndMatchFrame(feedForwardLinearAcceleration);

      command.getIncludingFrame(tempOrientation, tempAngularVelocity, feedForwardAngularAcceleration);
      yoDesiredOrientation.setAndMatchFrame(tempOrientation);
      yoDesiredAngularVelocity.setAndMatchFrame(tempAngularVelocity);
      yoFeedForwardAngularAcceleration.setAndMatchFrame(feedForwardAngularAcceleration);
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

      yoDesiredOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      yoDesiredAngularVelocity.getFrameTupleIncludingFrame(tempAngularVelocity);
      yoFeedForwardAngularAcceleration.getFrameTupleIncludingFrame(feedForwardAngularAcceleration);

      accelerationControlModule.doPositionControl(tempPosition, tempOrientation, tempLinearVelocity, tempAngularVelocity, feedForwardLinearAcceleration,
            feedForwardAngularAcceleration, base);
      accelerationControlModule.getAcceleration(desiredSpatialAcceleration);
      desiredSpatialAcceleration.changeBodyFrameNoRelativeAcceleration(endEffectorFrame);
      desiredSpatialAcceleration.changeFrameNoRelativeMotion(endEffectorFrame);

      updatePositionVisualization();
      updateOrientationVisualization();

      output.setSpatialAcceleration(desiredSpatialAcceleration);
   }

   private void updatePositionVisualization()
   {
      desiredSpatialAcceleration.getLinearPart(desiredLinearAcceleration);
      yoDesiredLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);

      tempPosition.setToZero(accelerationControlModule.getTrackingFrame());
      yoCurrentPosition.setAndMatchFrame(tempPosition);

      accelerationControlModule.getEndEffectorCurrentLinearVelocity(tempLinearVelocity);
      yoCurrentLinearVelocity.setAndMatchFrame(tempLinearVelocity);
   }

   private void updateOrientationVisualization()
   {
      desiredSpatialAcceleration.getAngularPart(desiredAngularAcceleration);
      yoDesiredAngularAcceleration.setAndMatchFrame(desiredAngularAcceleration);

      tempOrientation.setToZero(accelerationControlModule.getTrackingFrame());
      yoCurrentOrientation.setAndMatchFrame(tempOrientation);

      accelerationControlModule.getEndEffectorCurrentAngularVelocity(tempAngularVelocity);
      yoCurrentAngularVelocity.setAndMatchFrame(tempAngularVelocity);

      yoDesiredOrientation.get(tempAxisAngle);
      yoDesiredRotationVector.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      yoDesiredRotationVector.scale(tempAxisAngle.getAngle());

      yoCurrentOrientation.get(tempAxisAngle);
      yoCurrentRotationVector.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      yoCurrentRotationVector.scale(tempAxisAngle.getAngle());
   }

   public void computeAchievedAcceleration()
   {
      spatialAccelerationCalculator.getRelativeAcceleration(base, endEffector, endEffectorAchievedAcceleration);
      endEffectorAchievedAcceleration.getAngularPart(achievedAngularAcceleration);
      endEffectorAchievedAcceleration.getLinearPart(achievedLinearAcceleration);

      yoAchievedAngularAcceleration.setAndMatchFrame(achievedAngularAcceleration);
      yoAchievedLinearAcceleration.setAndMatchFrame(achievedLinearAcceleration);
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   @Override
   public SpatialAccelerationCommand getOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return output;
   }
}
