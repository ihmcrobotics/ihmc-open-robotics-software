package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.ANGULAR_ACCELERATION;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.ANGULAR_TORQUE;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.ANGULAR_VELOCITY;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space.ROTATION_VECTOR;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ACHIEVED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.CURRENT;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.DESIRED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ERROR;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ERROR_CUMULATED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.ERROR_INTEGRATED;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.FEEDBACK;
import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type.FEEDFORWARD;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualTorqueCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.math.filters.AlphaFilteredYoMutableFrameVector3D;
import us.ihmc.robotics.math.filters.RateLimitedYoMutableFrameVector3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class OrientationFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoBoolean isEnabled;

   private final FrameQuaternionBasics yoDesiredOrientation;
   private final FrameQuaternionBasics yoCurrentOrientation;
   private final FrameQuaternionBasics yoErrorOrientation;

   private final FrameQuaternionBasics yoErrorOrientationCumulated;

   private final FrameVector3DBasics yoDesiredRotationVector;
   private final FrameVector3DBasics yoCurrentRotationVector;
   private final FrameVector3DBasics yoErrorRotationVector;

   private final FrameVector3DBasics yoErrorRotationVectorIntegrated;

   private final FrameVector3DBasics yoDesiredAngularVelocity;
   private final FrameVector3DBasics yoCurrentAngularVelocity;
   private final FrameVector3DBasics yoErrorAngularVelocity;
   private final AlphaFilteredYoMutableFrameVector3D yoFilteredErrorAngularVelocity;
   private final FrameVector3DBasics yoFeedForwardAngularVelocity;
   private final FrameVector3DBasics yoFeedbackAngularVelocity;
   private final RateLimitedYoMutableFrameVector3D rateLimitedFeedbackAngularVelocity;

   private final FrameVector3DBasics yoDesiredAngularAcceleration;
   private final FrameVector3DBasics yoFeedForwardAngularAcceleration;
   private final FrameVector3DBasics yoFeedbackAngularAcceleration;
   private final RateLimitedYoMutableFrameVector3D rateLimitedFeedbackAngularAcceleration;
   private final FrameVector3DBasics yoAchievedAngularAcceleration;

   private final FrameVector3DBasics yoDesiredAngularTorque;
   private final FrameVector3DBasics yoFeedForwardAngularTorque;
   private final FrameVector3DBasics yoFeedbackAngularTorque;
   private final RateLimitedYoMutableFrameVector3D rateLimitedFeedbackAngularTorque;

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameQuaternion errorOrientationCumulated = new FrameQuaternion();

   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAngularVelocity = new FrameVector3D();

   private final FrameVector3D desiredAngularAcceleration = new FrameVector3D();
   private final FrameVector3D feedForwardAngularAction = new FrameVector3D();
   private final FrameVector3D feedForwardAngularAcceleration = new FrameVector3D();
   private final FrameVector3D achievedAngularAcceleration = new FrameVector3D();

   private final FrameVector3D desiredAngularTorque = new FrameVector3D();

   private final Twist currentTwist = new Twist();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final SpatialAccelerationCommand inverseDynamicsOutput = new SpatialAccelerationCommand();
   private final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();
   private final VirtualTorqueCommand virtualModelControlOutput = new VirtualTorqueCommand();
   private final MomentumRateCommand virtualModelControlRootOutput = new MomentumRateCommand();

   private final YoPID3DGains gains;
   private final Matrix3D tempGainMatrix = new Matrix3D();

   private final RigidBodyAccelerationProvider rigidBodyAccelerationProvider;

   private RigidBodyBasics base;
   private ReferenceFrame controlBaseFrame;
   private ReferenceFrame angularGainsFrame;

   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics endEffector;
   private final MovingReferenceFrame endEffectorFrame;

   private final double dt;
   private final boolean isRootBody;
   private final boolean computeIntegralTerm;

   public OrientationFeedbackController(RigidBodyBasics endEffector, WholeBodyControlCoreToolbox toolbox, FeedbackControllerToolbox feedbackControllerToolbox,
                                        YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;
      FeedbackControllerSettings settings = toolbox.getFeedbackControllerSettings();
      if (settings != null)
         computeIntegralTerm = settings.enableIntegralTerm();
      else
         computeIntegralTerm = true;

      if (toolbox.getRootJoint() != null)
      {
         this.rootBody = toolbox.getRootJoint().getSuccessor();
         isRootBody = this.endEffector.getName().equals(rootBody.getName());
      }
      else
      {
         isRootBody = false;
         rootBody = null;
      }

      rigidBodyAccelerationProvider = toolbox.getRigidBodyAccelerationProvider();

      String endEffectorName = endEffector.getName();
      registry = new YoVariableRegistry(endEffectorName + "OrientationFBController");
      dt = toolbox.getControlDT();
      gains = feedbackControllerToolbox.getOrientationGains(endEffector, computeIntegralTerm);
      YoDouble maximumRate = gains.getYoMaximumFeedbackRate();

      endEffectorFrame = endEffector.getBodyFixedFrame();

      isEnabled = new YoBoolean(endEffectorName + "IsOrientationFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredOrientation = feedbackControllerToolbox.getOrientation(endEffector, DESIRED, isEnabled);
      yoCurrentOrientation = feedbackControllerToolbox.getOrientation(endEffector, CURRENT, isEnabled);
      yoErrorOrientation = feedbackControllerToolbox.getOrientation(endEffector, ERROR, isEnabled);

      yoErrorOrientationCumulated = computeIntegralTerm ? feedbackControllerToolbox.getOrientation(endEffector, ERROR_CUMULATED, isEnabled) : null;

      yoDesiredRotationVector = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, ROTATION_VECTOR, isEnabled);
      yoCurrentRotationVector = feedbackControllerToolbox.getDataVector(endEffector, CURRENT, ROTATION_VECTOR, isEnabled);
      yoErrorRotationVector = feedbackControllerToolbox.getDataVector(endEffector, ERROR, ROTATION_VECTOR, isEnabled);

      yoErrorRotationVectorIntegrated = computeIntegralTerm ? feedbackControllerToolbox.getDataVector(endEffector, ERROR_INTEGRATED, ROTATION_VECTOR, isEnabled)
            : null;

      yoDesiredAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, ANGULAR_VELOCITY, isEnabled);

      if (toolbox.isEnableInverseDynamicsModule() || toolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, CURRENT, ANGULAR_VELOCITY, isEnabled);
         yoErrorAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, ERROR, ANGULAR_VELOCITY, isEnabled);
         DoubleProvider breakFrequency = feedbackControllerToolbox.getErrorVelocityFilterBreakFrequency(endEffectorName);
         if (breakFrequency != null)
            yoFilteredErrorAngularVelocity = feedbackControllerToolbox.getAlphaFilteredDataVector(endEffector, ERROR, ANGULAR_VELOCITY, dt, breakFrequency,
                                                                                                  isEnabled);
         else
            yoFilteredErrorAngularVelocity = null;

         if (toolbox.isEnableInverseDynamicsModule())
         {
            yoDesiredAngularAcceleration = feedbackControllerToolbox.getDataVector(endEffector, DESIRED, ANGULAR_ACCELERATION, isEnabled);
            yoFeedForwardAngularAcceleration = feedbackControllerToolbox.getDataVector(endEffector, FEEDFORWARD, ANGULAR_ACCELERATION, isEnabled);
            yoFeedbackAngularAcceleration = feedbackControllerToolbox.getDataVector(endEffector, FEEDBACK, ANGULAR_ACCELERATION, isEnabled);
            rateLimitedFeedbackAngularAcceleration = feedbackControllerToolbox.getRateLimitedDataVector(endEffector, FEEDBACK, ANGULAR_ACCELERATION, dt,
                                                                                                        maximumRate, isEnabled);
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
            yoFeedForwardAngularTorque = feedbackControllerToolbox.getDataVector(endEffector, FEEDFORWARD, ANGULAR_TORQUE, isEnabled);
            yoFeedbackAngularTorque = feedbackControllerToolbox.getDataVector(endEffector, FEEDBACK, ANGULAR_TORQUE, isEnabled);
            rateLimitedFeedbackAngularTorque = feedbackControllerToolbox.getRateLimitedDataVector(endEffector, FEEDBACK, ANGULAR_TORQUE, dt, maximumRate,
                                                                                                  isEnabled);
         }
         else
         {
            yoDesiredAngularTorque = null;
            yoFeedForwardAngularTorque = null;
            yoFeedbackAngularTorque = null;
            rateLimitedFeedbackAngularTorque = null;
         }
      }
      else
      {
         yoCurrentAngularVelocity = null;
         yoErrorAngularVelocity = null;
         yoFilteredErrorAngularVelocity = null;

         yoDesiredAngularAcceleration = null;
         yoFeedForwardAngularAcceleration = null;
         yoFeedbackAngularAcceleration = null;
         rateLimitedFeedbackAngularAcceleration = null;
         yoAchievedAngularAcceleration = null;

         yoDesiredAngularTorque = null;
         yoFeedForwardAngularTorque = null;
         yoFeedbackAngularTorque = null;
         rateLimitedFeedbackAngularTorque = null;
      }

      if (toolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, FEEDBACK, ANGULAR_VELOCITY, isEnabled);
         yoFeedForwardAngularVelocity = feedbackControllerToolbox.getDataVector(endEffector, FEEDFORWARD, ANGULAR_ACCELERATION, isEnabled);
         rateLimitedFeedbackAngularVelocity = feedbackControllerToolbox.getRateLimitedDataVector(endEffector, FEEDBACK, ANGULAR_VELOCITY, dt, maximumRate,
                                                                                                 isEnabled);
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

      yoDesiredOrientation.setIncludingFrame(command.getReferenceOrientation());
      yoDesiredOrientation.getRotationVector(yoDesiredRotationVector);
      yoDesiredAngularVelocity.setIncludingFrame(command.getReferenceAngularVelocity());
      yoDesiredAngularVelocity.checkReferenceFrameMatch(yoDesiredOrientation);
      if (yoFeedForwardAngularVelocity != null)
      {
         yoFeedForwardAngularVelocity.setIncludingFrame(command.getReferenceAngularVelocity());
         yoFeedForwardAngularVelocity.checkReferenceFrameMatch(yoDesiredOrientation);
      }
      if (yoFeedForwardAngularAcceleration != null)
      {
         yoFeedForwardAngularAcceleration.setIncludingFrame(command.getReferenceAngularAcceleration());
         yoFeedForwardAngularAcceleration.checkReferenceFrameMatch(yoDesiredOrientation);
      }
      if (yoFeedForwardAngularTorque != null)
      {
         yoFeedForwardAngularTorque.setIncludingFrame(command.getReferenceTorque());
         yoFeedForwardAngularTorque.checkReferenceFrameMatch(yoDesiredOrientation);
      }
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
      if (yoFilteredErrorAngularVelocity != null)
         yoFilteredErrorAngularVelocity.reset();
      if (yoErrorOrientationCumulated != null)
         yoErrorOrientationCumulated.setToZero(worldFrame);
      if (yoErrorRotationVectorIntegrated != null)
         yoErrorRotationVectorIntegrated.setToZero(worldFrame);
   }

   private final FrameVector3D proportionalFeedback = new FrameVector3D();
   private final FrameVector3D derivativeFeedback = new FrameVector3D();
   private final FrameVector3D integralFeedback = new FrameVector3D();

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled())
         return;

      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);
      feedForwardAngularAcceleration.setIncludingFrame(yoFeedForwardAngularAcceleration);
      feedForwardAngularAcceleration.changeFrame(endEffectorFrame);

      desiredAngularAcceleration.setIncludingFrame(proportionalFeedback);
      desiredAngularAcceleration.add(derivativeFeedback);
      desiredAngularAcceleration.add(integralFeedback);
      desiredAngularAcceleration.clipToMaxLength(gains.getMaximumFeedback());
      yoFeedbackAngularAcceleration.setIncludingFrame(desiredAngularAcceleration);
      yoFeedbackAngularAcceleration.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularAcceleration.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularAcceleration.update();
      desiredAngularAcceleration.setIncludingFrame(rateLimitedFeedbackAngularAcceleration);

      desiredAngularAcceleration.changeFrame(endEffectorFrame);
      desiredAngularAcceleration.add(feedForwardAngularAcceleration);

      yoDesiredAngularAcceleration.setIncludingFrame(desiredAngularAcceleration);
      yoDesiredAngularAcceleration.changeFrame(trajectoryFrame);

      inverseDynamicsOutput.setAngularAcceleration(endEffectorFrame, desiredAngularAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      inverseKinematicsOutput.setProperties(inverseDynamicsOutput);
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      feedForwardAngularVelocity.setIncludingFrame(yoFeedForwardAngularVelocity);
      computeProportionalTerm(proportionalFeedback);
      computeIntegralTerm(integralFeedback);

      desiredAngularVelocity.setIncludingFrame(proportionalFeedback);
      desiredAngularVelocity.add(integralFeedback);
      desiredAngularVelocity.clipToMaxLength(gains.getMaximumFeedback());
      yoFeedbackAngularVelocity.setIncludingFrame(desiredAngularVelocity);
      yoFeedbackAngularVelocity.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularVelocity.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularVelocity.update();
      desiredAngularVelocity.setIncludingFrame(rateLimitedFeedbackAngularVelocity);

      desiredAngularVelocity.add(feedForwardAngularVelocity);

      yoDesiredAngularVelocity.setIncludingFrame(desiredAngularVelocity);
      yoDesiredAngularVelocity.changeFrame(trajectoryFrame);

      desiredAngularVelocity.changeFrame(endEffectorFrame);
      inverseKinematicsOutput.setAngularVelocity(endEffectorFrame, desiredAngularVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      if (!isEnabled())
         return;

      computeFeedbackTorque();

      if (isRootBody)
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
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      feedForwardAngularAction.setIncludingFrame(yoFeedForwardAngularTorque);
      feedForwardAngularAction.changeFrame(endEffectorFrame);

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);

      desiredAngularTorque.setIncludingFrame(proportionalFeedback);
      desiredAngularTorque.add(derivativeFeedback);
      desiredAngularTorque.add(integralFeedback);
      desiredAngularTorque.clipToMaxLength(gains.getMaximumFeedback());
      yoFeedbackAngularTorque.setIncludingFrame(desiredAngularTorque);
      yoFeedbackAngularTorque.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularTorque.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularTorque.update();
      desiredAngularTorque.setIncludingFrame(rateLimitedFeedbackAngularTorque);

      desiredAngularTorque.changeFrame(endEffectorFrame);
      desiredAngularTorque.add(feedForwardAngularAction);

      yoDesiredAngularTorque.setIncludingFrame(desiredAngularTorque);
      yoDesiredAngularTorque.changeFrame(trajectoryFrame);
   }

   @Override
   public void computeAchievedAcceleration()
   {
      achievedAngularAcceleration.setIncludingFrame(rigidBodyAccelerationProvider.getRelativeAcceleration(base, endEffector).getAngularPart());
      yoAchievedAngularAcceleration.setIncludingFrame(achievedAngularAcceleration);
      yoAchievedAngularAcceleration.changeFrame(yoDesiredOrientation.getReferenceFrame());
   }

   /**
    * Computes the feedback term resulting from the error in orientation:<br>
    * x<sub>FB</sub> = kp * &theta;<sub>error</sub><br>
    * where &theta;<sub>error</sub> is a rotation vector representing the current error in orientation.
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
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      yoCurrentOrientation.setToZero(endEffectorFrame);
      yoCurrentOrientation.changeFrame(trajectoryFrame);
      yoCurrentOrientation.getRotationVector(yoCurrentRotationVector);

      desiredOrientation.setIncludingFrame(yoDesiredOrientation);
      desiredOrientation.changeFrame(endEffectorFrame);

      desiredOrientation.normalizeAndLimitToPi();
      desiredOrientation.getRotationVector(feedbackTermToPack);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumProportionalError());

      yoErrorRotationVector.setIncludingFrame(feedbackTermToPack);
      yoErrorRotationVector.changeFrame(trajectoryFrame);
      yoErrorOrientation.setRotationVectorIncludingFrame(yoErrorRotationVector);

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
    * This method also updates {@link #yoCurrentAngularVelocity} and {@link #yoErrorAngularVelocity}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   public void computeDerivativeTerm(FrameVector3D feedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      endEffectorFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      yoCurrentAngularVelocity.setIncludingFrame(currentTwist.getAngularPart());
      yoCurrentAngularVelocity.changeFrame(trajectoryFrame);

      feedbackTermToPack.setToZero(trajectoryFrame);
      feedbackTermToPack.sub(yoDesiredAngularVelocity, yoCurrentAngularVelocity);
      feedbackTermToPack.changeFrame(endEffectorFrame);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumDerivativeError());

      if (yoFilteredErrorAngularVelocity != null)
      {
         // If the trajectory frame changed reset the filter.
         if (yoFilteredErrorAngularVelocity.getReferenceFrame() != trajectoryFrame)
         {
            yoFilteredErrorAngularVelocity.setReferenceFrame(trajectoryFrame);
            yoFilteredErrorAngularVelocity.reset();
         }
         feedbackTermToPack.changeFrame(trajectoryFrame);
         yoErrorAngularVelocity.setIncludingFrame(feedbackTermToPack);
         yoFilteredErrorAngularVelocity.update();
         feedbackTermToPack.set(yoFilteredErrorAngularVelocity);
      }
      else
      {
         yoErrorAngularVelocity.setIncludingFrame(feedbackTermToPack);
      }
      yoErrorAngularVelocity.changeFrame(trajectoryFrame);

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
      if (!computeIntegralTerm)
      {
         feedbackTermToPack.setToZero(endEffectorFrame);
         return;
      }

      double maximumIntegralError = gains.getMaximumIntegralError();
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      if (maximumIntegralError < 1.0e-5)
      {
         feedbackTermToPack.setToZero(endEffectorFrame);
         yoErrorOrientationCumulated.setToZero(trajectoryFrame);
         yoErrorRotationVectorIntegrated.setToZero(trajectoryFrame);
         return;
      }

      // If the trajectory frame changed reset the integration.
      if (yoErrorOrientationCumulated.getReferenceFrame() != trajectoryFrame)
      {
         yoErrorOrientationCumulated.setToZero(trajectoryFrame);
         yoErrorRotationVectorIntegrated.setToZero(trajectoryFrame);
      }

      errorOrientationCumulated.setIncludingFrame(yoErrorOrientationCumulated);
      errorOrientationCumulated.multiply(yoErrorOrientation);
      yoErrorOrientationCumulated.set(errorOrientationCumulated);
      errorOrientationCumulated.normalizeAndLimitToPi();

      errorOrientationCumulated.getRotationVector(feedbackTermToPack);
      feedbackTermToPack.scale(dt);
      feedbackTermToPack.changeFrame(endEffectorFrame);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(maximumIntegralError);
      yoErrorRotationVectorIntegrated.setIncludingFrame(feedbackTermToPack);
      yoErrorRotationVectorIntegrated.changeFrame(trajectoryFrame);

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
      return (isRootBody) ? virtualModelControlRootOutput : virtualModelControlOutput;
   }
}
