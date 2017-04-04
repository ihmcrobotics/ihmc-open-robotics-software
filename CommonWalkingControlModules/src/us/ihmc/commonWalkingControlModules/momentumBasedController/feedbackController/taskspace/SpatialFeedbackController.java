package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
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
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class SpatialFeedbackController implements FeedbackControllerInterface
{
   private final YoVariableRegistry registry;

   private final BooleanYoVariable isEnabled;

   private final YoFramePoint yoDesiredPosition;
   private final YoFramePoint yoCurrentPosition;

   private final YoFrameQuaternion yoDesiredOrientation;
   private final YoFrameQuaternion yoCurrentOrientation;

   private final YoFrameVector yoDesiredLinearVelocity;
   private final YoFrameVector yoCurrentLinearVelocity;

   private final YoFrameVector yoDesiredAngularVelocity;
   private final YoFrameVector yoCurrentAngularVelocity;

   private final YoFrameVector yoFeedForwardLinearVelocity;
   private final YoFrameVector yoFeedbackLinearVelocity;

   private final YoFrameVector yoFeedForwardAngularVelocity;
   private final YoFrameVector yoFeedbackAngularVelocity;

   private final YoFrameVector yoFeedForwardLinearAcceleration;
   private final YoFrameVector yoFeedForwardAngularAcceleration;

   private final YoFrameVector yoDesiredLinearAcceleration;
   private final YoFrameVector yoAchievedLinearAcceleration;

   private final YoFrameVector yoDesiredAngularAcceleration;
   private final YoFrameVector yoAchievedAngularAcceleration;

   private final YoFrameVector yoDesiredRotationVector;
   private final YoFrameVector yoCurrentRotationVector;

   private final SpatialAccelerationVector endEffectorAchievedAcceleration = new SpatialAccelerationVector();
   private final FrameVector achievedAngularAcceleration = new FrameVector();
   private final FrameVector achievedLinearAcceleration = new FrameVector();

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameVector desiredLinearVelocity = new FrameVector();
   private final FrameVector desiredLinearAcceleration = new FrameVector();

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector desiredAngularAcceleration = new FrameVector();

   private final Twist desiredSpatialVelocity = new Twist();
   private final SpatialAccelerationVector desiredSpatialAcceleration = new SpatialAccelerationVector();

   private final AxisAngle tempAxisAngle = new AxisAngle();

   private final SpatialAccelerationCommand inverseDynamicsOutput = new SpatialAccelerationCommand();
   private final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();

   private final YoSE3PIDGainsInterface gains;
   private final RigidBodySpatialAccelerationControlModule accelerationControlModule;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private RigidBody base;

   private final RigidBody endEffector;
   private final ReferenceFrame endEffectorFrame;
   private final YoSE3OffsetFrame controlFrame;

   public SpatialFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
                                    YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;
      spatialAccelerationCalculator = toolbox.getSpatialAccelerationCalculator();

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "SpatialFBController");
      TwistCalculator twistCalculator = toolbox.getTwistCalculator();
      double dt = toolbox.getControlDT();
      gains = feedbackControllerToolbox.getSE3PIDGains(endEffector);
      controlFrame = feedbackControllerToolbox.getControlFrame(endEffector);

      endEffectorFrame = endEffector.getBodyFixedFrame();

      isEnabled = new BooleanYoVariable(endEffectorName + "isSpatialFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredPosition = feedbackControllerToolbox.getOrCreatePosition(endEffector, Type.DESIRED);
      yoCurrentPosition = feedbackControllerToolbox.getOrCreatePosition(endEffector, Type.CURRENT);

      yoDesiredOrientation = feedbackControllerToolbox.getOrCreateOrientation(endEffector, Type.DESIRED);
      yoCurrentOrientation = feedbackControllerToolbox.getOrCreateOrientation(endEffector, Type.CURRENT);

      yoDesiredRotationVector = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.ROTATION_VECTOR);
      yoCurrentRotationVector = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.CURRENT, Space.ROTATION_VECTOR);

      yoDesiredLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.LINEAR_VELOCITY);
      yoDesiredAngularVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.ANGULAR_VELOCITY);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.CURRENT, Space.LINEAR_VELOCITY);
         yoCurrentAngularVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.CURRENT, Space.ANGULAR_VELOCITY);

         accelerationControlModule = new RigidBodySpatialAccelerationControlModule(endEffectorName, twistCalculator, endEffector, controlFrame, dt, gains,
                                                                                   registry);
         yoFeedForwardLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDFORWARD, Space.LINEAR_ACCELERATION);
         yoDesiredLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.LINEAR_ACCELERATION);
         yoAchievedLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.ACHIEVED, Space.LINEAR_ACCELERATION);

         yoFeedForwardAngularAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDFORWARD, Space.ANGULAR_ACCELERATION);
         yoDesiredAngularAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.ANGULAR_ACCELERATION);
         yoAchievedAngularAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.ACHIEVED, Space.ANGULAR_ACCELERATION);
      }
      else
      {
         yoCurrentLinearVelocity = null;
         yoCurrentAngularVelocity = null;

         accelerationControlModule = null;
         yoFeedForwardLinearAcceleration = null;
         yoDesiredLinearAcceleration = null;
         yoAchievedLinearAcceleration = null;

         yoFeedForwardAngularAcceleration = null;
         yoDesiredAngularAcceleration = null;
         yoAchievedAngularAcceleration = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         yoFeedForwardLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDFORWARD, Space.LINEAR_VELOCITY);
         yoFeedbackLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDBACK, Space.LINEAR_VELOCITY);

         yoFeedForwardAngularVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDFORWARD, Space.ANGULAR_ACCELERATION);
         yoFeedbackAngularVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDBACK, Space.ANGULAR_VELOCITY);
      }
      else
      {
         yoFeedForwardLinearVelocity = null;
         yoFeedbackLinearVelocity = null;

         yoFeedForwardAngularVelocity = null;
         yoFeedbackAngularVelocity = null;
      }

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(SpatialFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();
      inverseDynamicsOutput.set(command.getSpatialAccelerationCommand());
      inverseKinematicsOutput.setProperties(command.getSpatialAccelerationCommand());

      gains.set(command.getGains());

      command.getControlFramePoseIncludingFrame(tempPosition, tempOrientation);
      controlFrame.setOffsetToParent(tempPosition, tempOrientation);

      command.getIncludingFrame(tempPosition, tempLinearVelocity, feedForwardLinearAcceleration);
      yoDesiredPosition.setAndMatchFrame(tempPosition);
      yoDesiredLinearVelocity.setAndMatchFrame(tempLinearVelocity);
      if (yoFeedForwardLinearVelocity != null)
         yoFeedForwardLinearVelocity.setAndMatchFrame(tempLinearVelocity);
      if (yoFeedForwardLinearAcceleration != null)
         yoFeedForwardLinearAcceleration.setAndMatchFrame(feedForwardLinearAcceleration);

      command.getIncludingFrame(tempOrientation, tempAngularVelocity, feedForwardAngularAcceleration);
      yoDesiredOrientation.setAndMatchFrame(tempOrientation);
      yoDesiredAngularVelocity.setAndMatchFrame(tempAngularVelocity);
      if (yoFeedForwardAngularVelocity != null)
         yoFeedForwardAngularVelocity.setAndMatchFrame(tempAngularVelocity);
      if (yoFeedForwardAngularAcceleration != null)
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
      if (accelerationControlModule != null)
         accelerationControlModule.reset();
   }

   @Override
   public void computeInverseDynamics()
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

      inverseDynamicsOutput.setSpatialAcceleration(desiredSpatialAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      yoDesiredPosition.getFrameTupleIncludingFrame(tempPosition);
      yoFeedForwardLinearVelocity.getFrameTupleIncludingFrame(tempLinearVelocity);
      tempLinearVelocity.changeFrame(controlFrame);

      tempPosition.changeFrame(controlFrame);
      double[] kp = gains.getPositionGains().getProportionalGains();
      desiredLinearVelocity.setToZero(controlFrame);
      desiredLinearVelocity.set(kp[0] * tempPosition.getX(), kp[1] * tempPosition.getY(), kp[2] * tempPosition.getZ());
      yoFeedbackLinearVelocity.setAndMatchFrame(desiredLinearVelocity);

      desiredLinearVelocity.add(tempLinearVelocity);
      yoDesiredLinearVelocity.setAndMatchFrame(desiredLinearVelocity);

      yoDesiredOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      yoFeedForwardAngularVelocity.getFrameTupleIncludingFrame(tempAngularVelocity);
      tempAngularVelocity.changeFrame(controlFrame);

      tempOrientation.changeFrame(controlFrame);
      tempOrientation.getAxisAngle(tempAxisAngle);
      kp = gains.getOrientationGains().getProportionalGains();
      desiredAngularVelocity.setToZero(controlFrame);
      desiredAngularVelocity.set(kp[0] * tempAxisAngle.getX(), kp[1] * tempAxisAngle.getY(), kp[2] * tempAxisAngle.getZ());
      desiredAngularVelocity.scale(AngleTools.trimAngleMinusPiToPi(tempAxisAngle.getAngle()));
      yoFeedbackAngularVelocity.setAndMatchFrame(desiredAngularVelocity);

      desiredSpatialVelocity.setToZero(endEffectorFrame, base.getBodyFixedFrame(), controlFrame);
      desiredSpatialVelocity.setAngularPart(desiredAngularVelocity);
      desiredSpatialVelocity.setLinearPart(desiredLinearVelocity);
      desiredSpatialVelocity.changeFrame(endEffectorFrame);

      inverseKinematicsOutput.setSpatialVelocity(desiredSpatialVelocity);

      // Update visualization variables
      tempPosition.setToZero(controlFrame);
      yoCurrentPosition.setAndMatchFrame(tempPosition);

      tempOrientation.setToZero(controlFrame);
      yoCurrentOrientation.setAndMatchFrame(tempOrientation);

      desiredAngularVelocity.add(tempAngularVelocity);
      yoDesiredAngularVelocity.setAndMatchFrame(desiredAngularVelocity);

      yoDesiredOrientation.get(tempAxisAngle);
      yoDesiredRotationVector.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      yoDesiredRotationVector.scale(tempAxisAngle.getAngle());

      yoCurrentOrientation.get(tempAxisAngle);
      yoCurrentRotationVector.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      yoCurrentRotationVector.scale(tempAxisAngle.getAngle());
   }

   @Override
   public void computeVirtualModelControl()
   {
      computeInverseDynamics();
   }

   private void updatePositionVisualization()
   {
      desiredSpatialAcceleration.getLinearPart(desiredLinearAcceleration);
      yoDesiredLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);

      tempPosition.setToZero(controlFrame);
      yoCurrentPosition.setAndMatchFrame(tempPosition);

      accelerationControlModule.getEndEffectorCurrentLinearVelocity(tempLinearVelocity);
      yoCurrentLinearVelocity.setAndMatchFrame(tempLinearVelocity);
   }

   private void updateOrientationVisualization()
   {
      desiredSpatialAcceleration.getAngularPart(desiredAngularAcceleration);
      yoDesiredAngularAcceleration.setAndMatchFrame(desiredAngularAcceleration);

      tempOrientation.setToZero(controlFrame);
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
   public SpatialAccelerationCommand getInverseDynamicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseDynamicsOutput;
   }

   @Override
   public SpatialVelocityCommand getInverseKinematicsOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return inverseKinematicsOutput;
   }

   @Override
   public SpatialAccelerationCommand getVirtualModelControlOutput()
   {
      return getInverseDynamicsOutput();
   }

   @Override
   public String toString()
   {
      return getClass().getSimpleName() + ": endEffector = " + endEffector;
   }
}
