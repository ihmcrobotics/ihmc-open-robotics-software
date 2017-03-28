package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodyLinearAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
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

   private final YoFrameVector yoFeedForwardLinearVelocity;
   private final YoFrameVector yoFeedForwardLinearAcceleration;

   private final YoFrameVector yoFeedbackLinearVelocity;

   private final YoFrameVector yoDesiredLinearAcceleration;
   private final YoFrameVector yoAchievedLinearAcceleration;

   private final FrameVector achievedLinearAcceleration = new FrameVector();

   private final FramePoint tempPosition = new FramePoint();
   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameVector desiredLinearVelocity = new FrameVector();
   private final FrameVector desiredLinearAcceleration = new FrameVector();

   private final PointAccelerationCommand inverseDynamicsOutput = new PointAccelerationCommand();
   private final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();

   private final YoPositionPIDGainsInterface gains;
   private final YoSE3OffsetFrame controlFrame;
   private final RigidBodyLinearAccelerationControlModule accelerationControlModule;
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
      controlFrame = feedbackControllerToolbox.getControlFrame(endEffector);

      isEnabled = new BooleanYoVariable(endEffectorName + "isPointFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredPosition = feedbackControllerToolbox.getOrCreatePosition(endEffector, Type.DESIRED);
      yoCurrentPosition = feedbackControllerToolbox.getOrCreatePosition(endEffector, Type.CURRENT);

      yoDesiredLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.LINEAR_VELOCITY);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.CURRENT, Space.LINEAR_VELOCITY);

         accelerationControlModule = new RigidBodyLinearAccelerationControlModule(endEffectorName, twistCalculator, endEffector, controlFrame, dt, gains, registry);
         yoFeedForwardLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDFORWARD, Space.LINEAR_ACCELERATION);
         yoDesiredLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.DESIRED, Space.LINEAR_ACCELERATION);
         yoAchievedLinearAcceleration = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.ACHIEVED, Space.LINEAR_ACCELERATION);
      }
      else
      {
         yoCurrentLinearVelocity = null;

         accelerationControlModule = null;
         yoFeedForwardLinearAcceleration = null;
         yoDesiredLinearAcceleration = null;
         yoAchievedLinearAcceleration = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         yoFeedForwardLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDFORWARD, Space.LINEAR_VELOCITY);
         yoFeedbackLinearVelocity = feedbackControllerToolbox.getOrCreateDataVector(endEffector, Type.FEEDBACK, Space.LINEAR_VELOCITY);
      }
      else
      {
         yoFeedForwardLinearVelocity = null;
         yoFeedbackLinearVelocity = null;
      }

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(PointFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();

      inverseDynamicsOutput.set(command.getPointAccelerationCommand());

      gains.set(command.getGains());

      command.getBodyFixedPointIncludingFrame(tempPosition);
      tempOrientation.setToZero(endEffector.getBodyFixedFrame());
      controlFrame.setOffsetToParent(tempPosition, tempOrientation);

      command.getIncludingFrame(tempPosition, tempLinearVelocity, feedForwardLinearAcceleration);
      yoDesiredPosition.setAndMatchFrame(tempPosition);
      yoDesiredLinearVelocity.setAndMatchFrame(tempLinearVelocity);
      if (yoFeedForwardLinearVelocity != null)
         yoFeedForwardLinearVelocity.setAndMatchFrame(tempLinearVelocity);
      if (yoFeedForwardLinearAcceleration != null)
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

      accelerationControlModule.compute(desiredLinearAcceleration, tempPosition, tempLinearVelocity, feedForwardLinearAcceleration, base);

      yoDesiredLinearAcceleration.setAndMatchFrame(desiredLinearAcceleration);

      tempPosition.setToZero(controlFrame);
      yoCurrentPosition.setAndMatchFrame(tempPosition);

      accelerationControlModule.getCurrentLinearVelocity(tempLinearVelocity);
      yoCurrentLinearVelocity.setAndMatchFrame(tempLinearVelocity);

      inverseDynamicsOutput.setLinearAcceleration(desiredLinearAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      inverseKinematicsOutput.setProperties(inverseDynamicsOutput);

      yoDesiredPosition.getFrameTupleIncludingFrame(tempPosition);
      yoFeedForwardLinearVelocity.getFrameTupleIncludingFrame(tempLinearVelocity);
      tempLinearVelocity.changeFrame(controlFrame);

      tempPosition.changeFrame(controlFrame);
      double[] kp = gains.getProportionalGains();
      desiredLinearVelocity.setToZero(controlFrame);
      desiredLinearVelocity.set(kp[0] * tempPosition.getX(), kp[1] * tempPosition.getY(), kp[2] * tempPosition.getZ());
      yoFeedbackLinearVelocity.setAndMatchFrame(desiredLinearVelocity);

      desiredLinearVelocity.add(tempLinearVelocity);
      yoDesiredLinearVelocity.setAndMatchFrame(desiredLinearVelocity);

      tempPosition.setToZero(controlFrame);
      yoCurrentPosition.setAndMatchFrame(tempPosition);

      inverseKinematicsOutput.setLinearVelocity(endEffector.getBodyFixedFrame(), base.getBodyFixedFrame(), desiredLinearVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      computeInverseDynamics();
   }

   @Override
   public void computeAchievedAcceleration()
   {
      tempPosition.setToZero(controlFrame);
      tempPosition.changeFrame(endEffector.getBodyFixedFrame());
      spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(base, endEffector, tempPosition, achievedLinearAcceleration);
      yoAchievedLinearAcceleration.setAndMatchFrame(achievedLinearAcceleration);
   }

   @Override
   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   @Override
   public PointAccelerationCommand getInverseDynamicsOutput()
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
   public PointAccelerationCommand getVirtualModelControlOutput()
   {
      return getInverseDynamicsOutput();
   }
}
