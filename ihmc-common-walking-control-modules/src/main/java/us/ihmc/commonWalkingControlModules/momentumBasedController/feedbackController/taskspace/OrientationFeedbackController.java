package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import us.ihmc.commonWalkingControlModules.controlModules.YoOrientationFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerException;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBAlphaFilteredVector3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBQuaternion3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBRateLimitedVector3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBVector3D;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings.FilterVector3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox.appendIndex;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.*;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.*;

public class OrientationFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoBoolean isEnabled;

   private final FBQuaternion3D yoDesiredOrientation;
   private final FBQuaternion3D yoCurrentOrientation;
   private final FBQuaternion3D yoErrorOrientation;

   private final FBQuaternion3D yoErrorOrientationCumulated;

   private final FBVector3D yoDesiredRotationVector;
   private final FBVector3D yoCurrentRotationVector;
   private final FBVector3D yoErrorRotationVector;

   private final FBVector3D yoErrorRotationVectorIntegrated;

   private final FBVector3D yoDesiredAngularVelocity;
   private final FBVector3D yoCurrentAngularVelocity;
   private final FBVector3D yoErrorAngularVelocity;
   private final FBAlphaFilteredVector3D yoFilteredErrorAngularVelocity;
   private final FilterVector3D angularVelocityErrorFilter;
   private final FBVector3D yoFeedForwardAngularVelocity;
   private final FBVector3D yoFeedbackAngularVelocity;
   private final FBRateLimitedVector3D rateLimitedFeedbackAngularVelocity;
   private final FBVector3D yoAchievedAngularVelocity;

   private final FBVector3D yoDesiredAngularAcceleration;
   private final FBVector3D yoFeedForwardAngularAcceleration;
   private final FBVector3D yoFeedbackAngularAcceleration;
   private final FBRateLimitedVector3D rateLimitedFeedbackAngularAcceleration;
   private final FBVector3D yoAchievedAngularAcceleration;

   private final FBVector3D yoDesiredAngularTorque;
   private final FBVector3D yoFeedForwardAngularTorque;
   private final FBVector3D yoFeedbackAngularTorque;
   private final FBRateLimitedVector3D rateLimitedFeedbackAngularTorque;

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameQuaternion controlFrameOrientation = new FrameQuaternion();
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

   private final RigidBodyTwistProvider rigidBodyTwistProvider;
   private final RigidBodyAccelerationProvider rigidBodyAccelerationProvider;

   private RigidBodyBasics base;
   private ReferenceFrame controlBaseFrame;
   private ReferenceFrame angularGainsFrame;

   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics endEffector;
   private final YoOrientationFrame controlFrame;

   private final double dt;
   private final boolean isRootBody;
   private final boolean computeIntegralTerm;

   private final int controllerIndex;
   private int currentCommandId;

   public OrientationFeedbackController(RigidBodyBasics endEffector,
                                        WholeBodyControlCoreToolbox ccToolbox,
                                        FeedbackControllerToolbox fbToolbox,
                                        YoRegistry parentRegistry)
   {
      this(endEffector, 0, ccToolbox, fbToolbox, parentRegistry);
   }

   public OrientationFeedbackController(RigidBodyBasics endEffector,
                                        int controllerIndex,
                                        WholeBodyControlCoreToolbox ccToolbox,
                                        FeedbackControllerToolbox fbToolbox,
                                        YoRegistry parentRegistry)
   {
      this.endEffector = endEffector;
      this.controllerIndex = controllerIndex;
      FeedbackControllerSettings settings = ccToolbox.getFeedbackControllerSettings();
      if (settings != null)
         computeIntegralTerm = settings.enableIntegralTerm();
      else
         computeIntegralTerm = true;

      if (ccToolbox.getRootJoint() != null)
      {
         this.rootBody = ccToolbox.getRootJoint().getSuccessor();
         isRootBody = this.endEffector.getName().equals(rootBody.getName());
      }
      else
      {
         isRootBody = false;
         rootBody = null;
      }

      rigidBodyTwistProvider = ccToolbox.getRigidBodyTwistCalculator();
      rigidBodyAccelerationProvider = ccToolbox.getRigidBodyAccelerationProvider();

      String endEffectorName = endEffector.getName();
      dt = ccToolbox.getControlDT();
      gains = fbToolbox.getOrCreateOrientationGains(endEffector, controllerIndex, computeIntegralTerm, true);
      YoDouble maximumRate = gains.getYoMaximumFeedbackRate();

      controlFrame = fbToolbox.getOrCreateOrientationFeedbackControlFrame(endEffector, controllerIndex, true);

      isEnabled = new YoBoolean(appendIndex(endEffectorName, controllerIndex) + "IsOrientationFBControllerEnabled", fbToolbox.getRegistry());
      isEnabled.set(false);

      yoDesiredOrientation = fbToolbox.getOrCreateOrientationData(endEffector, controllerIndex, DESIRED, isEnabled, true);
      yoCurrentOrientation = fbToolbox.getOrCreateOrientationData(endEffector, controllerIndex, CURRENT, isEnabled, true);
      yoErrorOrientation = fbToolbox.getOrCreateOrientationData(endEffector, controllerIndex, ERROR, isEnabled, false);

      yoDesiredRotationVector = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, ROTATION_VECTOR, isEnabled, true);
      yoCurrentRotationVector = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, CURRENT, ROTATION_VECTOR, isEnabled, true);
      yoErrorRotationVector = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ERROR, ROTATION_VECTOR, isEnabled, false);

      if (computeIntegralTerm)
      {
         yoErrorOrientationCumulated = fbToolbox.getOrCreateOrientationData(endEffector, controllerIndex, ERROR_CUMULATED, isEnabled, false);
         yoErrorRotationVectorIntegrated = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ERROR_INTEGRATED, ROTATION_VECTOR, isEnabled, false);
      }
      else
      {
         yoErrorOrientationCumulated = null;
         yoErrorRotationVectorIntegrated = null;
      }

      yoDesiredAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, ANGULAR_VELOCITY, isEnabled, true);

      if (ccToolbox.isEnableInverseDynamicsModule() || ccToolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, CURRENT, ANGULAR_VELOCITY, isEnabled, true);
         yoErrorAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ERROR, ANGULAR_VELOCITY, isEnabled, false);
         DoubleProvider breakFrequency = fbToolbox.getErrorVelocityFilterBreakFrequency(endEffectorName);
         if (breakFrequency != null)
            yoFilteredErrorAngularVelocity = fbToolbox.getOrCreateAlphaFilteredVectorData3D(endEffector,
                                                                                            controllerIndex,
                                                                                            ERROR,
                                                                                            ANGULAR_VELOCITY,
                                                                                            dt,
                                                                                            breakFrequency,
                                                                                            isEnabled,
                                                                                            false);
         else
            yoFilteredErrorAngularVelocity = null;

         angularVelocityErrorFilter = fbToolbox.getOrCreateAngularVelocityErrorFilter(endEffector, controllerIndex, dt);

         if (ccToolbox.isEnableInverseDynamicsModule())
         {
            yoDesiredAngularAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, ANGULAR_ACCELERATION, isEnabled, true);
            yoFeedForwardAngularAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector,
                                                                                 controllerIndex,
                                                                                 FEEDFORWARD,
                                                                                 ANGULAR_ACCELERATION,
                                                                                 isEnabled,
                                                                                 false);
            yoFeedbackAngularAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDBACK, ANGULAR_ACCELERATION, isEnabled, false);
            rateLimitedFeedbackAngularAcceleration = fbToolbox.getOrCreateRateLimitedVectorData3D(endEffector,
                                                                                                  controllerIndex,
                                                                                                  FEEDBACK,
                                                                                                  ANGULAR_ACCELERATION,
                                                                                                  dt,
                                                                                                  maximumRate,
                                                                                                  isEnabled,
                                                                                                  false);
            yoAchievedAngularAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ACHIEVED, ANGULAR_ACCELERATION, isEnabled, true);
         }
         else
         {
            yoDesiredAngularAcceleration = null;
            yoFeedForwardAngularAcceleration = null;
            yoFeedbackAngularAcceleration = null;
            rateLimitedFeedbackAngularAcceleration = null;
            yoAchievedAngularAcceleration = null;
         }

         if (ccToolbox.isEnableVirtualModelControlModule())
         {
            yoDesiredAngularTorque = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, ANGULAR_TORQUE, isEnabled, true);
            yoFeedForwardAngularTorque = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDFORWARD, ANGULAR_TORQUE, isEnabled, false);
            yoFeedbackAngularTorque = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDBACK, ANGULAR_TORQUE, isEnabled, false);
            rateLimitedFeedbackAngularTorque = fbToolbox.getOrCreateRateLimitedVectorData3D(endEffector,
                                                                                            controllerIndex,
                                                                                            FEEDBACK,
                                                                                            ANGULAR_TORQUE,
                                                                                            dt,
                                                                                            maximumRate,
                                                                                            isEnabled,
                                                                                            false);
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
         angularVelocityErrorFilter = null;

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

      if (ccToolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDBACK, ANGULAR_VELOCITY, isEnabled, false);
         yoFeedForwardAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDFORWARD, ANGULAR_VELOCITY, isEnabled, false);
         rateLimitedFeedbackAngularVelocity = fbToolbox.getOrCreateRateLimitedVectorData3D(endEffector,
                                                                                           controllerIndex,
                                                                                           FEEDBACK,
                                                                                           ANGULAR_VELOCITY,
                                                                                           dt,
                                                                                           maximumRate,
                                                                                           isEnabled,
                                                                                           false);
         yoAchievedAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ACHIEVED, ANGULAR_VELOCITY, isEnabled, true);
      }
      else
      {
         yoFeedbackAngularVelocity = null;
         yoFeedForwardAngularVelocity = null;
         rateLimitedFeedbackAngularVelocity = null;
         yoAchievedAngularVelocity = null;
      }
   }

   public void submitFeedbackControlCommand(OrientationFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new FeedbackControllerException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      currentCommandId = command.getCommandId();
      base = command.getBase();
      controlBaseFrame = command.getControlBaseFrame();

      inverseDynamicsOutput.set(command.getSpatialAccelerationCommand());
      inverseKinematicsOutput.setProperties(command.getSpatialAccelerationCommand());
      virtualModelControlOutput.setProperties(command.getSpatialAccelerationCommand());

      gains.set(command.getGains());
      command.getSpatialAccelerationCommand().getSelectionMatrix(selectionMatrix);
      angularGainsFrame = command.getAngularGainsFrame();

      command.getControlFrameOrientationIncludingFrame(controlFrameOrientation);
      controlFrame.setRotationToParent(controlFrameOrientation);

      yoDesiredOrientation.setIncludingFrame(command.getReferenceOrientation());
      yoDesiredOrientation.getRotationVector(yoDesiredRotationVector);
      yoDesiredOrientation.setCommandId(currentCommandId);
      yoDesiredAngularVelocity.setIncludingFrame(command.getReferenceAngularVelocity());
      yoDesiredAngularVelocity.checkReferenceFrameMatch(yoDesiredOrientation);
      yoDesiredAngularVelocity.setCommandId(currentCommandId);
      if (yoFeedForwardAngularVelocity != null)
      {
         yoFeedForwardAngularVelocity.setIncludingFrame(command.getReferenceAngularVelocity());
         yoFeedForwardAngularVelocity.checkReferenceFrameMatch(yoDesiredOrientation);
         yoFeedForwardAngularVelocity.setCommandId(currentCommandId);
      }
      if (yoFeedForwardAngularAcceleration != null)
      {
         yoFeedForwardAngularAcceleration.setIncludingFrame(command.getReferenceAngularAcceleration());
         yoFeedForwardAngularAcceleration.checkReferenceFrameMatch(yoDesiredOrientation);
         yoFeedForwardAngularAcceleration.setCommandId(currentCommandId);
      }
      if (yoFeedForwardAngularTorque != null)
      {
         yoFeedForwardAngularTorque.setIncludingFrame(command.getReferenceTorque());
         yoFeedForwardAngularTorque.checkReferenceFrameMatch(yoDesiredOrientation);
         yoFeedForwardAngularTorque.setCommandId(currentCommandId);
      }
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   @Override
   public void initialize()
   { // TODO See SpatialFeedbackController.initialize()
      if (rateLimitedFeedbackAngularAcceleration != null)
         rateLimitedFeedbackAngularAcceleration.reset();
      if (rateLimitedFeedbackAngularVelocity != null)
         rateLimitedFeedbackAngularVelocity.reset();
      if (yoFilteredErrorAngularVelocity != null)
         yoFilteredErrorAngularVelocity.reset();
      if (angularVelocityErrorFilter != null)
         angularVelocityErrorFilter.reset();
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
      feedForwardAngularAcceleration.changeFrame(controlFrame);

      desiredAngularAcceleration.setIncludingFrame(proportionalFeedback);
      desiredAngularAcceleration.add(derivativeFeedback);
      desiredAngularAcceleration.add(integralFeedback);
      desiredAngularAcceleration.clipToMaxNorm(gains.getMaximumFeedback());
      yoFeedbackAngularAcceleration.setIncludingFrame(desiredAngularAcceleration);
      yoFeedbackAngularAcceleration.changeFrame(trajectoryFrame);
      yoFeedbackAngularAcceleration.setCommandId(currentCommandId);
      rateLimitedFeedbackAngularAcceleration.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularAcceleration.update();
      rateLimitedFeedbackAngularAcceleration.setCommandId(currentCommandId);
      desiredAngularAcceleration.setIncludingFrame(rateLimitedFeedbackAngularAcceleration);

      desiredAngularAcceleration.changeFrame(controlFrame);
      desiredAngularAcceleration.add(feedForwardAngularAcceleration);

      yoDesiredAngularAcceleration.setIncludingFrame(desiredAngularAcceleration);
      yoDesiredAngularAcceleration.changeFrame(trajectoryFrame);
      yoDesiredAngularAcceleration.setCommandId(currentCommandId);

      inverseDynamicsOutput.setAngularAcceleration(controlFrame, desiredAngularAcceleration);
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
      desiredAngularVelocity.clipToMaxNorm(gains.getMaximumFeedback());
      yoFeedbackAngularVelocity.setIncludingFrame(desiredAngularVelocity);
      yoFeedbackAngularVelocity.changeFrame(trajectoryFrame);
      yoFeedbackAngularVelocity.setCommandId(currentCommandId);
      rateLimitedFeedbackAngularVelocity.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularVelocity.update();
      rateLimitedFeedbackAngularVelocity.setCommandId(currentCommandId);
      desiredAngularVelocity.setIncludingFrame(rateLimitedFeedbackAngularVelocity);

      desiredAngularVelocity.add(feedForwardAngularVelocity);

      yoDesiredAngularVelocity.setIncludingFrame(desiredAngularVelocity);
      yoDesiredAngularVelocity.changeFrame(trajectoryFrame);
      yoDesiredAngularVelocity.setCommandId(currentCommandId);

      desiredAngularVelocity.changeFrame(controlFrame);
      inverseKinematicsOutput.setAngularVelocity(controlFrame, desiredAngularVelocity);
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
         virtualModelControlOutput.setAngularTorque(controlFrame, desiredAngularTorque);
      }
   }

   private void computeFeedbackTorque()
   {
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      feedForwardAngularAction.setIncludingFrame(yoFeedForwardAngularTorque);
      feedForwardAngularAction.changeFrame(controlFrame);

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);

      desiredAngularTorque.setIncludingFrame(proportionalFeedback);
      desiredAngularTorque.add(derivativeFeedback);
      desiredAngularTorque.add(integralFeedback);
      desiredAngularTorque.clipToMaxNorm(gains.getMaximumFeedback());
      yoFeedbackAngularTorque.setIncludingFrame(desiredAngularTorque);
      yoFeedbackAngularTorque.changeFrame(trajectoryFrame);
      yoFeedbackAngularTorque.setCommandId(currentCommandId);
      rateLimitedFeedbackAngularTorque.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularTorque.update();
      rateLimitedFeedbackAngularTorque.setCommandId(currentCommandId);
      desiredAngularTorque.setIncludingFrame(rateLimitedFeedbackAngularTorque);

      desiredAngularTorque.changeFrame(controlFrame);
      desiredAngularTorque.add(feedForwardAngularAction);

      yoDesiredAngularTorque.setIncludingFrame(desiredAngularTorque);
      yoDesiredAngularTorque.changeFrame(trajectoryFrame);
      yoDesiredAngularTorque.setCommandId(currentCommandId);
   }

   @Override
   public void computeAchievedAcceleration()
   {
      achievedAngularAcceleration.setIncludingFrame(rigidBodyAccelerationProvider.getRelativeAcceleration(base, endEffector).getAngularPart());
      yoAchievedAngularAcceleration.setIncludingFrame(achievedAngularAcceleration);
      yoAchievedAngularAcceleration.changeFrame(yoDesiredOrientation.getReferenceFrame());
      yoAchievedAngularAcceleration.setCommandId(currentCommandId);
   }

   @Override
   public void computeAchievedVelocity()
   {
      if (yoAchievedAngularVelocity == null)
         return;

      yoAchievedAngularVelocity.setIncludingFrame(rigidBodyTwistProvider.getRelativeTwist(base, endEffector).getAngularPart());
      yoAchievedAngularVelocity.changeFrame(yoDesiredOrientation.getReferenceFrame());
      yoAchievedAngularVelocity.setCommandId(currentCommandId);
   }

   /**
    * Computes the feedback term resulting from the error in orientation:<br>
    * x<sub>FB</sub> = kp * &theta;<sub>error</sub><br>
    * where &theta;<sub>error</sub> is a rotation vector representing the current error in orientation.
    * <p>
    * The desired orientation of the {@code controlFrame} is obtained from
    * {@link #yoDesiredOrientation}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentOrientation}, {@link #yoCurrentRotationVector}, and
    * {@link #yoErrorOrientation}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   private void computeProportionalTerm(FrameVector3D feedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      yoCurrentOrientation.setToZero(controlFrame);
      yoCurrentOrientation.changeFrame(trajectoryFrame);
      yoCurrentOrientation.setCommandId(currentCommandId);
      yoCurrentOrientation.getRotationVector(yoCurrentRotationVector);
      yoCurrentRotationVector.setCommandId(currentCommandId);

      desiredOrientation.setIncludingFrame(yoDesiredOrientation);
      desiredOrientation.changeFrame(controlFrame);

      desiredOrientation.normalizeAndLimitToPi();
      desiredOrientation.getRotationVector(feedbackTermToPack);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxNorm(gains.getMaximumProportionalError());

      yoErrorRotationVector.setIncludingFrame(feedbackTermToPack);
      yoErrorRotationVector.changeFrame(controlFrame);
      yoErrorRotationVector.setCommandId(currentCommandId);
      yoErrorOrientation.setRotationVectorIncludingFrame(yoErrorRotationVector);
      yoErrorRotationVector.setCommandId(currentCommandId);

      if (angularGainsFrame != null)
         feedbackTermToPack.changeFrame(angularGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      gains.getProportionalGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);

      feedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the error in angular velocity:<br>
    * x<sub>FB</sub> = kd * (&omega;<sub>desired</sub> - &omega;<sub>current</sub>)
    * <p>
    * The desired angular velocity of the {@code controlFrame} relative to the {@code base} is
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

      controlFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      yoCurrentAngularVelocity.setIncludingFrame(currentTwist.getAngularPart());
      yoCurrentAngularVelocity.changeFrame(trajectoryFrame);
      yoCurrentAngularVelocity.setCommandId(currentCommandId);

      feedbackTermToPack.setToZero(trajectoryFrame);
      feedbackTermToPack.sub(yoDesiredAngularVelocity, yoCurrentAngularVelocity);
      feedbackTermToPack.changeFrame(controlFrame);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxNorm(gains.getMaximumDerivativeError());

      feedbackTermToPack.changeFrame(trajectoryFrame);
      yoErrorAngularVelocity.setIncludingFrame(feedbackTermToPack);

      if (yoFilteredErrorAngularVelocity != null)
      {
         // If the trajectory frame changed reset the filter.
         if (yoFilteredErrorAngularVelocity.getReferenceFrame() != trajectoryFrame)
         {
            yoFilteredErrorAngularVelocity.setReferenceFrame(trajectoryFrame);
            yoFilteredErrorAngularVelocity.reset();
         }
         yoFilteredErrorAngularVelocity.update();
         yoFilteredErrorAngularVelocity.setCommandId(currentCommandId);
         feedbackTermToPack.set(yoFilteredErrorAngularVelocity);
      }

      if (angularVelocityErrorFilter != null)
         angularVelocityErrorFilter.apply(feedbackTermToPack, feedbackTermToPack);

      yoErrorAngularVelocity.changeFrame(trajectoryFrame);
      yoErrorAngularVelocity.setCommandId(currentCommandId);

      feedbackTermToPack.changeFrame(angularGainsFrame != null ? angularGainsFrame : controlFrame);

      gains.getDerivativeGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);

      feedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the integrated error in orientation:<br>
    * x<sub>FB</sub> = ki * &int;<sup>t</sup> &theta;<sub>error</sub>
    * <p>
    * The current error in orientation of the {@code controlFrame} is obtained from
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
         feedbackTermToPack.setToZero(controlFrame);
         return;
      }

      double maximumIntegralError = gains.getMaximumIntegralError();
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      if (maximumIntegralError < 1.0e-5)
      {
         feedbackTermToPack.setToZero(controlFrame);
         yoErrorOrientationCumulated.setToZero(trajectoryFrame);
         yoErrorOrientationCumulated.setCommandId(currentCommandId);
         yoErrorRotationVectorIntegrated.setToZero(trajectoryFrame);
         yoErrorRotationVectorIntegrated.setCommandId(currentCommandId);
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
      yoErrorOrientationCumulated.setCommandId(currentCommandId);
      errorOrientationCumulated.normalizeAndLimitToPi();

      errorOrientationCumulated.getRotationVector(feedbackTermToPack);
      feedbackTermToPack.scale(dt);
      feedbackTermToPack.changeFrame(controlFrame);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxNorm(maximumIntegralError);
      yoErrorRotationVectorIntegrated.setIncludingFrame(feedbackTermToPack);
      yoErrorRotationVectorIntegrated.changeFrame(trajectoryFrame);
      yoErrorRotationVectorIntegrated.setCommandId(currentCommandId);

      if (angularGainsFrame != null)
         feedbackTermToPack.changeFrame(angularGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      gains.getIntegralGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);

      feedbackTermToPack.changeFrame(controlFrame);
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
         throw new FeedbackControllerException("This controller is disabled.");
      return inverseDynamicsOutput;
   }

   @Override
   public SpatialVelocityCommand getInverseKinematicsOutput()
   {
      if (!isEnabled())
         throw new FeedbackControllerException("This controller is disabled.");
      return inverseKinematicsOutput;
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlOutput()
   {
      if (!isEnabled())
         throw new FeedbackControllerException("This controller is disabled.");
      return (isRootBody) ? virtualModelControlRootOutput : virtualModelControlOutput;
   }

   public int getControllerIndex()
   {
      return controllerIndex;
   }
}
