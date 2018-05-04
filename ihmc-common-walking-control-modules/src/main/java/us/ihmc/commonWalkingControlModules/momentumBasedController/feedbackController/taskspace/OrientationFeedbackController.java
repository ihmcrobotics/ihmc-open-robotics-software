package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualTorqueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.*;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.*;

public class OrientationFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoBoolean isEnabled;

   private final YoFrameQuaternion yoDesiredOrientation;
   private final YoFrameQuaternion yoCurrentOrientation;
   private final YoFrameQuaternion yoErrorOrientation;

   private final YoFrameQuaternion yoErrorOrientationCumulated;

   private final YoFrameVector3D yoDesiredRotationVector;
   private final YoFrameVector3D yoCurrentRotationVector;
   private final YoFrameVector3D yoErrorRotationVector;

   private final YoFrameVector3D yoErrorRotationVectorIntegrated;

   private final YoFrameVector3D yoDesiredAngularVelocity;
   private final YoFrameVector3D yoCurrentAngularVelocity;
   private final YoFrameVector3D yoErrorAngularVelocity;
   private final YoFrameVector3D yoFeedForwardAngularVelocity;
   private final YoFrameVector3D yoFeedbackAngularVelocity;
   private final RateLimitedYoFrameVector rateLimitedFeedbackAngularVelocity;

   private final YoFrameVector3D yoDesiredAngularAcceleration;
   private final YoFrameVector3D yoFeedForwardAngularAcceleration;
   private final YoFrameVector3D yoFeedbackAngularAcceleration;
   private final RateLimitedYoFrameVector rateLimitedFeedbackAngularAcceleration;
   private final YoFrameVector3D yoAchievedAngularAcceleration;

   private final YoFrameVector3D yoDesiredAngularTorque;
   private final YoFrameVector3D yoFeedbackAngularTorque;
   private final RateLimitedYoFrameVector rateLimitedFeedbackAngularTorque;

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameQuaternion currentOrientation = new FrameQuaternion();
   private final FrameQuaternion errorOrientationCumulated = new FrameQuaternion();

   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D currentAngularVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAngularVelocity = new FrameVector3D();

   private final FrameVector3D desiredAngularAcceleration = new FrameVector3D();
   private final FrameVector3D feedForwardAngularAcceleration = new FrameVector3D();
   private final FrameVector3D achievedAngularAcceleration = new FrameVector3D();

   private final FrameVector3D desiredAngularTorque = new FrameVector3D();

   private final Twist currentTwist = new Twist();
   private final SpatialAccelerationVector endEffectorAchievedAcceleration = new SpatialAccelerationVector();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final SpatialAccelerationCommand inverseDynamicsOutput = new SpatialAccelerationCommand();
   private final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();
   private final VirtualTorqueCommand virtualModelControlOutput = new VirtualTorqueCommand();
   private final MomentumRateCommand virtualModelControlRootOutput = new MomentumRateCommand();

   private final YoPID3DGains gains;
   private final Matrix3D tempGainMatrix = new Matrix3D();

   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private RigidBody base;
   private ReferenceFrame controlBaseFrame;
   private ReferenceFrame angularGainsFrame;

   private final RigidBody rootBody;
   private final RigidBody endEffector;
   private final MovingReferenceFrame endEffectorFrame;

   private final double dt;

   public OrientationFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
                                        YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;

      if (toolbox.getRootJoint() != null)
         rootBody = toolbox.getRootJoint().getSuccessor();
      else
         rootBody = null;

      spatialAccelerationCalculator = toolbox.getSpatialAccelerationCalculator();

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "OrientationFBController");
      dt = toolbox.getControlDT();
      gains = feedbackControllerToolbox.getOrientationGains(endEffector);
      YoDouble maximumRate = gains.getYoMaximumFeedbackRate();

      endEffectorFrame = endEffector.getBodyFixedFrame();

      isEnabled = new YoBoolean(endEffectorName + "IsOrientationFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredOrientation = feedbackControllerToolbox.getOrientation(endEffector, DESIRED, isEnabled);
      yoCurrentOrientation = feedbackControllerToolbox.getOrientation(endEffector, CURRENT, isEnabled);
      yoErrorOrientation = feedbackControllerToolbox.getOrientation(endEffector, ERROR, isEnabled);

      yoErrorOrientationCumulated = feedbackControllerToolbox.getOrientation(endEffector, ERROR_CUMULATED, isEnabled);

      yoDesiredRotationVector = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, ROTATION_VECTOR, isEnabled);
      yoCurrentRotationVector = feedbackControllerToolbox.getDataVector(endEffector, CURRENT, ROTATION_VECTOR, isEnabled);
      yoErrorRotationVector = feedbackControllerToolbox.getDataVector(endEffector, ERROR, ROTATION_VECTOR, isEnabled);

      yoErrorRotationVectorIntegrated = feedbackControllerToolbox.getDataVector(endEffector, ERROR_INTEGRATED, ROTATION_VECTOR, isEnabled);

      yoDesiredAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, ANGULAR_VELOCITY, isEnabled);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, CURRENT, ANGULAR_VELOCITY, isEnabled);
         yoErrorAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, ERROR, ANGULAR_VELOCITY, isEnabled);

         if (toolbox.isEnableInverseDynamicsModule())
         {
            yoDesiredAngularAcceleration = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, ANGULAR_ACCELERATION, isEnabled);
            yoFeedForwardAngularAcceleration = feedbackControllerToolbox.getDataVector(endEffector, FEEDFORWARD, ANGULAR_ACCELERATION, isEnabled);
            yoFeedbackAngularAcceleration = feedbackControllerToolbox.getDataVector(endEffector, FEEDBACK, ANGULAR_ACCELERATION, isEnabled);
            rateLimitedFeedbackAngularAcceleration = feedbackControllerToolbox
                  .getRateLimitedDataVector(endEffector, FEEDBACK, ANGULAR_ACCELERATION, dt, maximumRate, isEnabled);
            yoAchievedAngularAcceleration = feedbackControllerToolbox.getDataVector(endEffector, ACHIEVED, ANGULAR_ACCELERATION, isEnabled);
         }
         else
         {
            yoDesiredAngularAcceleration = null;
            yoFeedForwardAngularAcceleration = null;
            yoFeedbackAngularAcceleration = null;
            rateLimitedFeedbackAngularAcceleration = null;
            yoAchievedAngularAcceleration = null;
         }

         if (toolbox.isEnableVirtualModelControlModule())
         {
            yoDesiredAngularTorque = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, ANGULAR_TORQUE, isEnabled);
            yoFeedbackAngularTorque = feedbackControllerToolbox.getDataVector(endEffector, FEEDBACK, ANGULAR_TORQUE, isEnabled);
            rateLimitedFeedbackAngularTorque = feedbackControllerToolbox
                  .getRateLimitedDataVector(endEffector, FEEDBACK, ANGULAR_TORQUE, dt, maximumRate, isEnabled);
         }
         else
         {
            yoDesiredAngularTorque = null;
            yoFeedbackAngularTorque = null;
            rateLimitedFeedbackAngularTorque = null;
         }
      }
      else
      {
         yoCurrentAngularVelocity = null;
         yoErrorAngularVelocity = null;

         yoDesiredAngularAcceleration = null;
         yoFeedForwardAngularAcceleration = null;
         yoFeedbackAngularAcceleration = null;
         rateLimitedFeedbackAngularAcceleration = null;
         yoAchievedAngularAcceleration = null;

         yoDesiredAngularTorque = null;
         yoFeedbackAngularTorque = null;
         rateLimitedFeedbackAngularTorque = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, FEEDBACK, ANGULAR_VELOCITY, isEnabled);
         yoFeedForwardAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, FEEDFORWARD, ANGULAR_ACCELERATION, isEnabled);
         rateLimitedFeedbackAngularVelocity = feedbackControllerToolbox
               .getRateLimitedDataVector(endEffector, FEEDBACK, ANGULAR_VELOCITY, dt, maximumRate, isEnabled);
      }
      else
      {
         yoFeedbackAngularVelocity = null;
         yoFeedForwardAngularVelocity = null;
         rateLimitedFeedbackAngularVelocity = null;
      }

      parentRegistry.addChild(registry);
   }

   public void submitFeedbackControlCommand(OrientationFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new RuntimeException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      base = command.getBase();
      controlBaseFrame = command.getControlBaseFrame();

      inverseDynamicsOutput.set(command.getSpatialAccelerationCommand());
      inverseKinematicsOutput.setProperties(command.getSpatialAccelerationCommand());
      virtualModelControlOutput.setProperties(command.getSpatialAccelerationCommand());

      gains.set(command.getGains());
      command.getSpatialAccelerationCommand().getSelectionMatrix(selectionMatrix);
      angularGainsFrame = command.getAngularGainsFrame();
      command.getIncludingFrame(desiredOrientation, desiredAngularVelocity);
      command.getFeedForwardActionIncludingFrame(feedForwardAngularAcceleration);

      yoDesiredOrientation.setMatchingFrame(desiredOrientation);
      desiredOrientation.getRotationVector(yoDesiredRotationVector);

      yoDesiredAngularVelocity.setMatchingFrame(desiredAngularVelocity);

      if (yoFeedForwardAngularVelocity != null)
         yoFeedForwardAngularVelocity.setMatchingFrame(desiredAngularVelocity);

      if (yoFeedForwardAngularAcceleration != null)
         yoFeedForwardAngularAcceleration.setMatchingFrame(feedForwardAngularAcceleration);
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   @Override
   public void initialize()
   {
      if (rateLimitedFeedbackAngularAcceleration != null)
         rateLimitedFeedbackAngularAcceleration.reset();
      if (rateLimitedFeedbackAngularVelocity != null)
         rateLimitedFeedbackAngularVelocity.reset();
   }

   private final FrameVector3D proportionalFeedback = new FrameVector3D();
   private final FrameVector3D derivativeFeedback = new FrameVector3D();
   private final FrameVector3D integralFeedback = new FrameVector3D();

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled())
         return;

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);
      feedForwardAngularAcceleration.setIncludingFrame(yoFeedForwardAngularAcceleration);
      feedForwardAngularAcceleration.changeFrame(endEffectorFrame);

      desiredAngularAcceleration.setIncludingFrame(proportionalFeedback);
      desiredAngularAcceleration.add(derivativeFeedback);
      desiredAngularAcceleration.add(integralFeedback);
      desiredAngularAcceleration.clipToMaxLength(gains.getMaximumFeedback());
      yoFeedbackAngularAcceleration.setMatchingFrame(desiredAngularAcceleration);
      rateLimitedFeedbackAngularAcceleration.update();
      desiredAngularAcceleration.setIncludingFrame(rateLimitedFeedbackAngularAcceleration);

      desiredAngularAcceleration.changeFrame(endEffectorFrame);
      desiredAngularAcceleration.add(feedForwardAngularAcceleration);

      yoDesiredAngularAcceleration.setMatchingFrame(desiredAngularAcceleration);

      inverseDynamicsOutput.setAngularAcceleration(endEffectorFrame, desiredAngularAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      inverseKinematicsOutput.setProperties(inverseDynamicsOutput);

      feedForwardAngularVelocity.setIncludingFrame(yoFeedForwardAngularVelocity);
      computeProportionalTerm(proportionalFeedback);
      computeIntegralTerm(integralFeedback);

      desiredAngularVelocity.setIncludingFrame(proportionalFeedback);
      desiredAngularVelocity.add(integralFeedback);
      desiredAngularVelocity.clipToMaxLength(gains.getMaximumFeedback());
      yoFeedbackAngularVelocity.setMatchingFrame(desiredAngularVelocity);
      rateLimitedFeedbackAngularVelocity.update();
      desiredAngularVelocity.setIncludingFrame(rateLimitedFeedbackAngularVelocity);

      desiredAngularVelocity.add(feedForwardAngularVelocity);

      yoDesiredAngularVelocity.setMatchingFrame(desiredAngularVelocity);

      desiredAngularVelocity.changeFrame(endEffectorFrame);
      inverseKinematicsOutput.setAngularVelocity(endEffectorFrame, desiredAngularVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      if (!isEnabled())
         return;

      computeFeedbackTorque();

      if (endEffector.getName().equals(rootBody.getName()))
      {
         desiredAngularTorque.changeFrame(worldFrame);

         virtualModelControlRootOutput.setProperties(inverseDynamicsOutput);
         virtualModelControlRootOutput.setAngularMomentumRate(desiredAngularTorque);
      }
      else
      {
         virtualModelControlOutput.setProperties(inverseDynamicsOutput);
         virtualModelControlOutput.setAngularTorque(endEffectorFrame, desiredAngularTorque);
      }
   }

   private void computeFeedbackTorque()
   {
      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);

      desiredAngularTorque.setIncludingFrame(proportionalFeedback);
      desiredAngularTorque.add(derivativeFeedback);
      desiredAngularTorque.add(integralFeedback);
      desiredAngularTorque.clipToMaxLength(gains.getMaximumFeedback());
      yoFeedbackAngularTorque.setMatchingFrame(desiredAngularTorque);
      rateLimitedFeedbackAngularTorque.update();
      desiredAngularTorque.setIncludingFrame(rateLimitedFeedbackAngularTorque);

      desiredAngularTorque.changeFrame(endEffectorFrame);
      yoDesiredAngularTorque.setMatchingFrame(desiredAngularTorque);
   }

   @Override
   public void computeAchievedAcceleration()
   {
      spatialAccelerationCalculator.getRelativeAcceleration(base, endEffector, endEffectorAchievedAcceleration);
      endEffectorAchievedAcceleration.getAngularPart(achievedAngularAcceleration);

      yoAchievedAngularAcceleration.setMatchingFrame(achievedAngularAcceleration);
   }

   /**
    * Computes the feedback term resulting from the error in orientation:<br>
    * x<sub>FB</sub> = kp * &theta;<sub>error</sub><br>
    * where &theta;<sub>error</sub> is a rotation vector representing the current error in
    * orientation.
    * <p>
    * The desired orientation of the {@code endEffectorFrame} is obtained from
    * {@link #yoDesiredOrientation}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentOrientation}, {@link #yoCurrentRotationVector}, and
    * {@link #yoErrorPosition}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   private void computeProportionalTerm(FrameVector3D feedbackTermToPack)
   {
      currentOrientation.setToZero(endEffectorFrame);
      currentOrientation.changeFrame(worldFrame);
      yoCurrentOrientation.set(currentOrientation);
      yoCurrentOrientation.getRotationVector(yoCurrentRotationVector);

      desiredOrientation.setIncludingFrame(yoDesiredOrientation);
      desiredOrientation.changeFrame(endEffectorFrame);

      desiredOrientation.normalizeAndLimitToPi();
      desiredOrientation.getRotationVector(feedbackTermToPack);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumProportionalError());
      yoErrorRotationVector.setMatchingFrame(feedbackTermToPack);
      yoErrorOrientation.setRotationVector(yoErrorRotationVector);

      if (angularGainsFrame != null)
         feedbackTermToPack.changeFrame(angularGainsFrame);
      else
         feedbackTermToPack.changeFrame(endEffectorFrame);

      gains.getProportionalGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);

      feedbackTermToPack.changeFrame(endEffectorFrame);
   }

   /**
    * Computes the feedback term resulting from the error in angular velocity:<br>
    * x<sub>FB</sub> = kd * (&omega;<sub>desired</sub> - &omega;<sub>current</sub>)
    * <p>
    * The desired angular velocity of the {@code endEffectorFrame} relative to the {@code base} is
    * obtained from {@link #yoDesiredAngularVelocity}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentAngularVelocity} and
    * {@link #yoErrorAngularVelocity}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   public void computeDerivativeTerm(FrameVector3D feedbackTermToPack)
   {
      endEffectorFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      currentTwist.getAngularPart(currentAngularVelocity);
      currentAngularVelocity.changeFrame(worldFrame);
      yoCurrentAngularVelocity.set(currentAngularVelocity);

      desiredAngularVelocity.setIncludingFrame(yoDesiredAngularVelocity);

      feedbackTermToPack.setToZero(worldFrame);
      feedbackTermToPack.sub(desiredAngularVelocity, currentAngularVelocity);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumDerivativeError());
      yoErrorAngularVelocity.set(feedbackTermToPack);

      if (angularGainsFrame != null)
         feedbackTermToPack.changeFrame(angularGainsFrame);
      else
         feedbackTermToPack.changeFrame(endEffectorFrame);

      gains.getDerivativeGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);

      feedbackTermToPack.changeFrame(endEffectorFrame);
   }

   /**
    * Computes the feedback term resulting from the integrated error in orientation:<br>
    * x<sub>FB</sub> = ki * &int;<sup>t</sup> &theta;<sub>error</sub>
    * <p>
    * The current error in orientation of the {@code endEffectorFrame} is obtained from
    * {@link #yoErrorOrientation}.
    * </p>
    * <p>
    * This method also updates {@link #yoErrorOrientationCumulated} and
    * {@link #yoErrorRotationVectorIntegrated}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   public void computeIntegralTerm(FrameVector3D feedbackTermToPack)
   {
      double maximumIntegralError = gains.getMaximumIntegralError();

      if (maximumIntegralError < 1.0e-5)
      {
         feedbackTermToPack.setToZero(endEffectorFrame);
         yoErrorOrientationCumulated.setToZero();
         yoErrorRotationVectorIntegrated.setToZero();
         return;
      }

      errorOrientationCumulated.setIncludingFrame(yoErrorOrientationCumulated);
      errorOrientationCumulated.multiply(yoErrorOrientation);
      yoErrorOrientationCumulated.set(errorOrientationCumulated);
      errorOrientationCumulated.normalizeAndLimitToPi();

      errorOrientationCumulated.getRotationVector(feedbackTermToPack);
      feedbackTermToPack.scale(dt);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(maximumIntegralError);
      yoErrorRotationVectorIntegrated.set(feedbackTermToPack);

      if (angularGainsFrame != null)
         feedbackTermToPack.changeFrame(angularGainsFrame);
      else
         feedbackTermToPack.changeFrame(endEffectorFrame);

      gains.getIntegralGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);

      feedbackTermToPack.changeFrame(endEffectorFrame);
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
   public VirtualModelControlCommand<?> getVirtualModelControlOutput()
   {
      if (!isEnabled())
         throw new RuntimeException("This controller is disabled.");
      return (endEffector.getName().equals(rootBody.getName())) ? virtualModelControlRootOutput : virtualModelControlOutput;
   }
}
