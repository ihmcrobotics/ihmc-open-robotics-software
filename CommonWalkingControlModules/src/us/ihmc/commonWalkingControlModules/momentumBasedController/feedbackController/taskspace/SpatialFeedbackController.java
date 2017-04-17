package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.RateLimitedYoSpatialVector;
import us.ihmc.robotics.math.frames.YoFramePoseUsingQuaternions;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoSpatialVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class SpatialFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final BooleanYoVariable isEnabled;

   private final YoFramePoseUsingQuaternions yoDesiredPose;
   private final YoFramePoseUsingQuaternions yoCurrentPose;

   private final YoSpatialVector yoErrorVector;
   private final YoFrameQuaternion yoErrorOrientation;

   private final YoFrameVector yoErrorPositionIntegrated;
   private final YoFrameQuaternion yoErrorOrientationCumulated;
   private final YoFrameVector yoErrorRotationVectorIntegrated;

   private final YoSpatialVector yoDesiredVelocity;
   private final YoSpatialVector yoCurrentVelocity;
   private final YoSpatialVector yoErrorVelocity;
   private final YoSpatialVector yoFeedForwardVelocity;
   private final YoSpatialVector yoFeedbackVelocity;
   private final RateLimitedYoSpatialVector rateLimitedFeedbackVelocity;

   private final YoSpatialVector yoDesiredAcceleration;
   private final YoSpatialVector yoFeedForwardAcceleration;
   private final YoSpatialVector yoFeedbackAcceleration;
   private final RateLimitedYoSpatialVector rateLimitedFeedbackAcceleration;
   private final YoSpatialVector yoAchievedAcceleration;

   private final YoFrameVector yoDesiredRotationVector;
   private final YoFrameVector yoCurrentRotationVector;

   private final FramePoint desiredPosition = new FramePoint();
   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FramePose currentPose = new FramePose();
   private final FramePose desiredPose = new FramePose();

   private final FrameOrientation errorOrientationCumulated = new FrameOrientation();

   private final FrameVector desiredLinearVelocity = new FrameVector();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector currentLinearVelocity = new FrameVector();
   private final FrameVector currentAngularVelocity = new FrameVector();
   private final FrameVector feedForwardLinearVelocity = new FrameVector();
   private final FrameVector feedForwardAngularVelocity = new FrameVector();

   private final FrameVector desiredLinearAcceleration = new FrameVector();
   private final FrameVector desiredAngularAcceleration = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final FrameVector biasLinearAcceleration = new FrameVector();
   private final FrameVector achievedAngularAcceleration = new FrameVector();
   private final FrameVector achievedLinearAcceleration = new FrameVector();

   private final Twist currentTwist = new Twist();
   private final SpatialAccelerationVector endEffectorAchievedAcceleration = new SpatialAccelerationVector();

   private final SpatialAccelerationCommand inverseDynamicsOutput = new SpatialAccelerationCommand();
   private final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();

   private final YoSE3PIDGainsInterface gains;
   private final YoPositionPIDGainsInterface positionGains;
   private final YoOrientationPIDGainsInterface orientationGains;
   private final Matrix3DReadOnly kpLinear, kdLinear, kiLinear;
   private final Matrix3DReadOnly kpAngular, kdAngular, kiAngular;

   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   private RigidBody base;

   private final RigidBody endEffector;
   private final YoSE3OffsetFrame controlFrame;

   private final double dt;

   public SpatialFeedbackController(RigidBody endEffector, WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
                                    YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;

      twistCalculator = toolbox.getTwistCalculator();
      spatialAccelerationCalculator = toolbox.getSpatialAccelerationCalculator();

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "SpatialFBController");
      dt = toolbox.getControlDT();
      gains = feedbackControllerToolbox.getSE3PIDGains(endEffector);
      positionGains = gains.getPositionGains();
      orientationGains = gains.getOrientationGains();
      DoubleYoVariable maximumLinearRate = positionGains.getYoMaximumFeedbackRate();
      DoubleYoVariable maximumAngularRate = orientationGains.getYoMaximumFeedbackRate();

      kpLinear = positionGains.createProportionalGainMatrix();
      kdLinear = positionGains.createDerivativeGainMatrix();
      kiLinear = positionGains.createIntegralGainMatrix();

      kpAngular = orientationGains.createProportionalGainMatrix();
      kdAngular = orientationGains.createDerivativeGainMatrix();
      kiAngular = orientationGains.createIntegralGainMatrix();

      controlFrame = feedbackControllerToolbox.getControlFrame(endEffector);

      isEnabled = new BooleanYoVariable(endEffectorName + "isSpatialFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredPose = feedbackControllerToolbox.getPose(endEffector, Type.DESIRED);
      yoCurrentPose = feedbackControllerToolbox.getPose(endEffector, Type.CURRENT);
      YoFrameVector errorPosition = feedbackControllerToolbox.getDataVector(endEffector, Type.ERROR, Space.POSITION);
      YoFrameVector errorRotationVector = feedbackControllerToolbox.getDataVector(endEffector, Type.ERROR, Space.ROTATION_VECTOR);
      yoErrorVector = new YoSpatialVector(errorPosition, errorRotationVector);
      yoErrorOrientation = feedbackControllerToolbox.getOrientation(endEffector, Type.ERROR);
      yoErrorPositionIntegrated = feedbackControllerToolbox.getDataVector(endEffector, Type.ERROR_INTEGRATED, Space.POSITION);
      yoErrorOrientationCumulated = feedbackControllerToolbox.getOrientation(endEffector, Type.ERROR_CUMULATED);
      yoErrorRotationVectorIntegrated = feedbackControllerToolbox.getDataVector(endEffector, Type.ERROR_INTEGRATED, Space.ROTATION_VECTOR);

      yoDesiredRotationVector = feedbackControllerToolbox.getDataVector(endEffector, Type.DESIRED, Space.ROTATION_VECTOR);
      yoCurrentRotationVector = feedbackControllerToolbox.getDataVector(endEffector, Type.CURRENT, Space.ROTATION_VECTOR);

      yoDesiredVelocity = feedbackControllerToolbox.getVelocity(endEffector, Type.DESIRED);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentVelocity = feedbackControllerToolbox.getVelocity(endEffector, Type.CURRENT);
         yoErrorVelocity = feedbackControllerToolbox.getVelocity(endEffector, Type.ERROR);

         yoDesiredAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, Type.DESIRED);
         yoFeedForwardAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, Type.FEEDFORWARD);
         yoFeedbackAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, Type.FEEDBACK);
         rateLimitedFeedbackAcceleration = feedbackControllerToolbox.getRateLimitedAcceleration(endEffector, Type.FEEDBACK, dt, maximumLinearRate,
                                                                                                maximumAngularRate);
         yoAchievedAcceleration = feedbackControllerToolbox.getAcceleration(endEffector, Type.ACHIEVED);
      }
      else
      {
         yoCurrentVelocity = null;
         yoErrorVelocity = null;

         yoDesiredAcceleration = null;
         yoFeedForwardAcceleration = null;
         yoFeedbackAcceleration = null;
         rateLimitedFeedbackAcceleration = null;
         yoAchievedAcceleration = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackVelocity = feedbackControllerToolbox.getVelocity(endEffector, Type.FEEDBACK);
         yoFeedForwardVelocity = feedbackControllerToolbox.getVelocity(endEffector, Type.FEEDFORWARD);
         rateLimitedFeedbackVelocity = feedbackControllerToolbox.getRateLimitedVelocity(endEffector, Type.FEEDBACK, dt, maximumLinearRate, maximumAngularRate);
      }
      else
      {
         yoFeedbackVelocity = null;
         yoFeedForwardVelocity = null;
         rateLimitedFeedbackVelocity = null;
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

      command.getControlFramePoseIncludingFrame(desiredPosition, desiredOrientation);
      controlFrame.setOffsetToParent(desiredPosition, desiredOrientation);

      command.getIncludingFrame(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      command.getIncludingFrame(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      yoDesiredPose.setAndMatchFrame(desiredPosition, desiredOrientation);
      yoDesiredRotationVector.setAsRotationVector(yoDesiredPose.getOrientation());
      yoDesiredVelocity.setAndMatchFrame(desiredLinearVelocity, desiredAngularVelocity);

      if (yoFeedForwardVelocity != null)
         yoFeedForwardVelocity.setAndMatchFrame(desiredLinearVelocity, desiredAngularVelocity);

      if (yoFeedForwardAcceleration != null)
         yoFeedForwardAcceleration.setAndMatchFrame(feedForwardLinearAcceleration, feedForwardAngularAcceleration);
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   @Override
   public void initialize()
   {
      if (rateLimitedFeedbackAcceleration != null)
         rateLimitedFeedbackAcceleration.reset();
      if (rateLimitedFeedbackVelocity != null)
         rateLimitedFeedbackVelocity.reset();
   }

   private final FrameVector linearProportionalFeedback = new FrameVector();
   private final FrameVector linearDerivativeFeedback = new FrameVector();
   private final FrameVector linearIntegralFeedback = new FrameVector();

   private final FrameVector angularProportionalFeedback = new FrameVector();
   private final FrameVector angularDerivativeFeedback = new FrameVector();
   private final FrameVector angularIntegralFeedback = new FrameVector();

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled())
         return;

      computeProportionalTerm(linearProportionalFeedback, angularProportionalFeedback);
      computeDerivativeTerm(linearDerivativeFeedback, angularDerivativeFeedback);
      computeIntegralTerm(linearIntegralFeedback, angularIntegralFeedback);
      yoFeedForwardAcceleration.getIncludingFrame(feedForwardLinearAcceleration, feedForwardAngularAcceleration);
      feedForwardLinearAcceleration.changeFrame(controlFrame);
      feedForwardAngularAcceleration.changeFrame(controlFrame);

      desiredLinearAcceleration.setIncludingFrame(linearProportionalFeedback);
      desiredLinearAcceleration.add(linearDerivativeFeedback);
      desiredLinearAcceleration.add(linearIntegralFeedback);
      desiredLinearAcceleration.limitLength(positionGains.getMaximumFeedback());

      desiredAngularAcceleration.setIncludingFrame(angularProportionalFeedback);
      desiredAngularAcceleration.add(angularDerivativeFeedback);
      desiredAngularAcceleration.add(angularIntegralFeedback);
      desiredAngularAcceleration.limitLength(orientationGains.getMaximumFeedback());

      yoFeedbackAcceleration.setAndMatchFrame(desiredLinearAcceleration, desiredAngularAcceleration);
      rateLimitedFeedbackAcceleration.update();
      rateLimitedFeedbackAcceleration.getIncludingFrame(desiredLinearAcceleration, desiredAngularAcceleration);

      desiredLinearAcceleration.changeFrame(controlFrame);
      desiredLinearAcceleration.add(feedForwardLinearAcceleration);

      desiredAngularAcceleration.changeFrame(controlFrame);
      desiredAngularAcceleration.add(feedForwardAngularAcceleration);

      yoDesiredAcceleration.setAndMatchFrame(desiredLinearAcceleration, desiredAngularAcceleration);

      addCoriolisAcceleration(desiredLinearAcceleration);

      inverseDynamicsOutput.setSpatialAcceleration(controlFrame, desiredAngularAcceleration, desiredLinearAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      inverseKinematicsOutput.setProperties(inverseDynamicsOutput);

      yoFeedForwardVelocity.getIncludingFrame(feedForwardLinearVelocity, feedForwardAngularVelocity);
      computeProportionalTerm(linearProportionalFeedback, angularProportionalFeedback);
      computeIntegralTerm(linearIntegralFeedback, angularIntegralFeedback);

      desiredLinearVelocity.setIncludingFrame(linearProportionalFeedback);
      desiredLinearVelocity.add(linearIntegralFeedback);
      desiredLinearVelocity.limitLength(positionGains.getMaximumFeedback());

      desiredAngularVelocity.setIncludingFrame(angularProportionalFeedback);
      desiredAngularVelocity.add(angularIntegralFeedback);
      desiredAngularVelocity.limitLength(orientationGains.getMaximumFeedback());

      yoFeedbackVelocity.setAndMatchFrame(desiredLinearVelocity, desiredAngularVelocity);
      rateLimitedFeedbackVelocity.update();
      rateLimitedFeedbackVelocity.getIncludingFrame(desiredLinearVelocity, desiredAngularVelocity);

      desiredLinearVelocity.add(feedForwardLinearVelocity);
      desiredAngularVelocity.add(feedForwardAngularVelocity);

      yoDesiredVelocity.setAndMatchFrame(desiredLinearVelocity, desiredAngularVelocity);

      desiredLinearVelocity.changeFrame(controlFrame);
      desiredAngularVelocity.changeFrame(controlFrame);
      inverseKinematicsOutput.setSpatialVelocity(controlFrame, desiredAngularVelocity, desiredLinearVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      computeInverseDynamics();
   }

   public void computeAchievedAcceleration()
   {
      spatialAccelerationCalculator.getRelativeAcceleration(base, endEffector, endEffectorAchievedAcceleration);
      endEffectorAchievedAcceleration.changeFrameNoRelativeMotion(controlFrame);
      endEffectorAchievedAcceleration.getAngularPart(achievedAngularAcceleration);
      endEffectorAchievedAcceleration.getLinearPart(achievedLinearAcceleration);
      subtractCoriolisAcceleration(achievedLinearAcceleration);

      yoAchievedAcceleration.setAndMatchFrame(achievedLinearAcceleration, achievedAngularAcceleration);
   }

   /**
    * Computes the feedback term resulting from the error in position and orienation:<br>
    * x<sub>FB</sub><sup>linear</sup> = kp<sup>linear</sup> * (x<sub>desired</sub> -
    * x<sub>current</sub>)<br>
    * x<sub>FB</sub><sup>angular</sup> = kp<sup>angular</sup> * &theta;<sub>error</sub><br>
    * where &theta;<sub>error</sub> is a rotation vector representing the current error in
    * orientation.
    * <p>
    * The desired pose of the {@code controlFrame} is obtained from {@link #yoDesiredPose}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentPose}, {@link #yoErrorVector}, and
    * {@link #yoErrorOrientation}.
    * </p>
    * 
    * @param linearFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>linear</sup>. Modified.
    * @param angularFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>angular</sup>. Modified.
    */
   private void computeProportionalTerm(FrameVector linearFeedbackTermToPack, FrameVector angularFeedbackTermToPack)
   {
      currentPose.setToZero(controlFrame);
      currentPose.changeFrame(worldFrame);
      yoCurrentPose.set(currentPose);
      yoCurrentPose.getOrientation().getRotationVector(yoCurrentRotationVector);

      yoDesiredPose.getFramePoseIncludingFrame(desiredPose);
      desiredPose.changeFrame(controlFrame);

      desiredPose.normalizeQuaternionAndLimitToPi();
      desiredPose.getPositionIncludingFrame(linearFeedbackTermToPack);
      desiredPose.getRotationVectorIncludingFrame(angularFeedbackTermToPack);

      linearFeedbackTermToPack.limitLength(positionGains.getMaximumProportionalError());
      angularFeedbackTermToPack.limitLength(orientationGains.getMaximumProportionalError());

      yoErrorVector.setAndMatchFrame(linearFeedbackTermToPack, angularFeedbackTermToPack);
      yoErrorOrientation.setRotationVector(yoErrorVector.getYoAngularPart());

      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
      kpLinear.transform(linearFeedbackTermToPack.getVector());
      kpAngular.transform(angularFeedbackTermToPack.getVector());
   }

   /**
    * Computes the feedback term resulting from the error in linear velocity:<br>
    * x<sub>FB</sub><sup>linear</sup> = kd<sup>linear</sup> * (xDot<sub>desired</sub> -
    * xDot<sub>current</sub>)<br>
    * x<sub>FB</sub><sup>angular</sup> = kd<sup>angular</sup> * (&omega;<sub>desired</sub> -
    * &omega;<sub>current</sub>)
    * <p>
    * The desired velocity of the {@code controlFrame} relative to the {@code base} is obtained from
    * {@link #yoDesiredVelocity}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentVelocity} and {@link #yoErrorVelocity}.
    * </p>
    * 
    * @param linearFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>linear</sup>. Modified.
    * @param angularFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>angular</sup>. Modified.
    */
   private void computeDerivativeTerm(FrameVector linearFeedbackTermToPack, FrameVector angularFeedbackTermToPack)
   {
      twistCalculator.getRelativeTwist(base, endEffector, currentTwist);
      currentTwist.changeFrame(controlFrame);
      currentTwist.getLinearPart(currentLinearVelocity);
      currentTwist.getAngularPart(currentAngularVelocity);
      currentLinearVelocity.changeFrame(worldFrame);
      currentAngularVelocity.changeFrame(worldFrame);
      yoCurrentVelocity.setAndMatchFrame(currentLinearVelocity, currentAngularVelocity);

      yoDesiredVelocity.getIncludingFrame(desiredLinearVelocity, desiredAngularVelocity);

      linearFeedbackTermToPack.setToZero(worldFrame);
      angularFeedbackTermToPack.setToZero(worldFrame);
      linearFeedbackTermToPack.sub(desiredLinearVelocity, currentLinearVelocity);
      angularFeedbackTermToPack.sub(desiredAngularVelocity, currentAngularVelocity);
      linearFeedbackTermToPack.limitLength(positionGains.getMaximumDerivativeError());
      angularFeedbackTermToPack.limitLength(orientationGains.getMaximumDerivativeError());
      yoErrorVelocity.set(linearFeedbackTermToPack, angularFeedbackTermToPack);

      linearFeedbackTermToPack.changeFrame(controlFrame);
      angularFeedbackTermToPack.changeFrame(controlFrame);
      kdLinear.transform(linearFeedbackTermToPack.getVector());
      kdAngular.transform(angularFeedbackTermToPack.getVector());
   }

   /**
    * Computes the feedback term resulting from the integrated error in position:<br>
    * x<sub>FB</sub><sup>linear</sup> = ki<sup>linear</sup> * &int;<sup>t</sup> (x<sub>desired</sub>
    * - x<sub>current</sub>)<br>
    * x<sub>FB</sub><sup>angular</sup> = ki<sup>angular</sup> * &int;<sup>t</sup>
    * &theta;<sub>error</sub>
    * <p>
    * The current error in position and orientation of the {@code controlFrame} is obtained from
    * {@link #yoErrorVector} and {@link #yoErrorOrientation} respectively.
    * </p>
    * <p>
    * This method also updates {@link #yoErrorPositionIntegrated},
    * {@link #yoErrorOrientationCumulated}, and {@link #yoErrorRotationVectorIntegrated}.
    * </p>
    * 
    * @param linearFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>linear</sup>. Modified.
    * @param angularFeedbackTermToPack the value of the feedback term
    *           x<sub>FB</sub><sup>angular</sup>. Modified.
    */
   private void computeIntegralTerm(FrameVector linearFeedbackTermToPack, FrameVector angularFeedbackTermToPack)
   {
      double maximumLinearIntegralError = positionGains.getMaximumIntegralError();

      if (maximumLinearIntegralError < 1.0e-5)
      {
         linearFeedbackTermToPack.setToZero(controlFrame);
         yoErrorPositionIntegrated.setToZero();
      }
      else
      {
         yoErrorVector.getLinearPartIncludingFrame(linearFeedbackTermToPack);
         linearFeedbackTermToPack.scale(dt);
         linearFeedbackTermToPack.add(yoErrorPositionIntegrated.getFrameTuple());
         linearFeedbackTermToPack.limitLength(maximumLinearIntegralError);
         yoErrorPositionIntegrated.set(linearFeedbackTermToPack);

         linearFeedbackTermToPack.changeFrame(controlFrame);
         kiLinear.transform(linearFeedbackTermToPack.getVector());
      }

      double maximumAngularIntegralError = orientationGains.getMaximumIntegralError();

      if (maximumAngularIntegralError < 1.0e-5)
      {
         angularFeedbackTermToPack.setToZero(controlFrame);
         yoErrorOrientationCumulated.setToZero();
         yoErrorRotationVectorIntegrated.setToZero();
      }
      else
      {
         yoErrorOrientationCumulated.getFrameOrientationIncludingFrame(errorOrientationCumulated);
         errorOrientationCumulated.multiply(yoErrorOrientation.getFrameOrientation());
         yoErrorOrientationCumulated.set(errorOrientationCumulated);
         errorOrientationCumulated.normalizeAndLimitToPiMinusPi();

         errorOrientationCumulated.getRotationVectorIncludingFrame(angularFeedbackTermToPack);
         angularFeedbackTermToPack.scale(dt);
         angularFeedbackTermToPack.limitLength(maximumAngularIntegralError);
         yoErrorRotationVectorIntegrated.set(angularFeedbackTermToPack);

         angularFeedbackTermToPack.changeFrame(controlFrame);
         kiAngular.transform(angularFeedbackTermToPack.getVector());
      }
   }

   /**
    * Computes and adds the bias acceleration resulting from the combination of the current linear
    * and angular velocity of the control frame.
    * <p>
    * This is needed when going from a linear acceleration expressed in an inertial frame to a
    * moving frame attached to the end-effector.
    * </p>
    * <p>
    * Intuitively, the Coriolis acceleration only appears when measuring the acceleration from a
    * moving frame, here a frame attache to the end-effector.
    * </p>
    * 
    * @param linearAccelerationToModify the linear acceleration vector to which the bias is to be
    *           subtracted. Its frame is changed to {@code controlFrame}. Modified.
    */
   private void addCoriolisAcceleration(FrameVector linearAccelerationToModify)
   {
      twistCalculator.getTwistOfBody(endEffector, currentTwist);
      currentTwist.changeBodyFrameNoRelativeTwist(controlFrame);
      currentTwist.changeFrame(controlFrame);

      currentTwist.getAngularPart(currentAngularVelocity);
      currentTwist.getLinearPart(currentLinearVelocity);

      biasLinearAcceleration.setToZero(controlFrame);
      biasLinearAcceleration.cross(currentLinearVelocity, currentAngularVelocity);
      linearAccelerationToModify.changeFrame(controlFrame);
      linearAccelerationToModify.add(biasLinearAcceleration);
   }

   /**
    * Computes and subtracts the bias acceleration resulting from the combination of the current
    * linear and angular velocity of the control frame.
    * <p>
    * This is needed when going from a linear acceleration expressed in a moving frame attached to
    * the end-effector to an inertial frame.
    * </p>
    * <p>
    * Intuitively, the Coriolis acceleration only appears when measuring the acceleration from a
    * moving frame, here a frame attache to the end-effector.
    * </p>
    * 
    * @param linearAccelerationToModify the linear acceleration vector to which the bias is to be
    *           added. Its frame is changed to {@code worldFrame}. Modified.
    */
   private void subtractCoriolisAcceleration(FrameVector linearAccelerationToModify)
   {
      twistCalculator.getTwistOfBody(endEffector, currentTwist);
      currentTwist.changeBodyFrameNoRelativeTwist(controlFrame);
      currentTwist.changeFrame(controlFrame);

      currentTwist.getAngularPart(currentAngularVelocity);
      currentTwist.getLinearPart(currentLinearVelocity);

      biasLinearAcceleration.setToZero(controlFrame);
      biasLinearAcceleration.cross(currentLinearVelocity, currentAngularVelocity);
      linearAccelerationToModify.changeFrame(controlFrame);
      linearAccelerationToModify.sub(biasLinearAcceleration);
      linearAccelerationToModify.changeFrame(worldFrame);
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
