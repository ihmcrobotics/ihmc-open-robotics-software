package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox.appendIndex;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.LINEAR_ACCELERATION;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.LINEAR_FORCE;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.LINEAR_VELOCITY;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.POSITION;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.CURRENT;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.DESIRED;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.ERROR;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.ERROR_INTEGRATED;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.FEEDBACK;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.FEEDFORWARD;

import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controlModules.YoTranslationFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerException;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBAlphaFilteredVector3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBPoint3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBRateLimitedVector3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.data.FBVector3D;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerInterface;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PointFeedbackController implements FeedbackControllerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoBoolean isEnabled;

   private final FBPoint3D yoDesiredPosition;
   private final FBPoint3D yoCurrentPosition;
   private final FBVector3D yoErrorPosition;

   private final FBVector3D yoErrorPositionIntegrated;

   private final FBVector3D yoDesiredLinearVelocity;
   private final FBVector3D yoCurrentLinearVelocity;
   private final FBVector3D yoErrorLinearVelocity;
   private final FBAlphaFilteredVector3D yoFilteredErrorLinearVelocity;
   private final FBVector3D yoFeedForwardLinearVelocity;
   private final FBVector3D yoFeedbackLinearVelocity;
   private final FBRateLimitedVector3D rateLimitedFeedbackLinearVelocity;

   private final FBVector3D yoDesiredLinearAcceleration;
   private final FBVector3D yoFeedForwardLinearAcceleration;
   private final FBVector3D yoFeedbackLinearAcceleration;
   private final FBRateLimitedVector3D rateLimitedFeedbackLinearAcceleration;
   private final FBVector3D yoAchievedLinearAcceleration;

   private final FBVector3D yoDesiredLinearForce;
   private final FBVector3D yoFeedForwardLinearForce;
   private final FBVector3D yoFeedbackLinearForce;
   private final FBRateLimitedVector3D rateLimitedFeedbackLinearForce;

   private final FramePoint3D desiredPosition = new FramePoint3D();

   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D currentLinearVelocity = new FrameVector3D();
   private final FrameVector3D currentAngularVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardLinearVelocity = new FrameVector3D();

   private final FrameVector3D desiredLinearAcceleration = new FrameVector3D();
   private final FrameVector3D feedForwardLinearForce = new FrameVector3D();
   private final FrameVector3D feedForwardLinearAcceleration = new FrameVector3D();
   private final FrameVector3D biasLinearAcceleration = new FrameVector3D();
   private final FrameVector3D achievedLinearAcceleration = new FrameVector3D();

   private final FrameVector3D desiredLinearForce = new FrameVector3D();

   private final Twist currentTwist = new Twist();

   private final SpatialAccelerationCommand inverseDynamicsOutput = new SpatialAccelerationCommand();
   private final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();
   private final VirtualForceCommand virtualModelControlOutput = new VirtualForceCommand();
   private final MomentumRateCommand virtualModelControlRootOutput = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private final YoPID3DGains gains;
   private final Matrix3D tempGainMatrix = new Matrix3D();
   private final YoTranslationFrame controlFrame;

   private final RigidBodyAccelerationProvider rigidBodyAccelerationProvider;

   private RigidBodyBasics base;
   private ReferenceFrame controlBaseFrame;
   private ReferenceFrame linearGainsFrame;

   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics endEffector;

   private final double dt;
   private final boolean isRootBody;
   private final boolean computeIntegralTerm;

   private final int controllerIndex;
   private int currentCommandId;

   public PointFeedbackController(RigidBodyBasics endEffector,
                                  WholeBodyControlCoreToolbox ccToolbox,
                                  FeedbackControllerToolbox fbToolbox,
                                  YoRegistry parentRegistry)
   {
      this(endEffector, 0, ccToolbox, fbToolbox, parentRegistry);
   }

   public PointFeedbackController(RigidBodyBasics endEffector,
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

      rigidBodyAccelerationProvider = ccToolbox.getRigidBodyAccelerationProvider();

      String endEffectorName = endEffector.getName();
      dt = ccToolbox.getControlDT();
      gains = fbToolbox.getOrCreatePositionGains(endEffector, controllerIndex, computeIntegralTerm, true);
      YoDouble maximumRate = gains.getYoMaximumFeedbackRate();

      controlFrame = fbToolbox.getOrCreatePointFeedbackControlFrame(endEffector, controllerIndex, true);

      isEnabled = new YoBoolean(appendIndex(endEffectorName, controllerIndex) + "isPointFBControllerEnabled", fbToolbox.getRegistry());
      isEnabled.set(false);

      yoDesiredPosition = fbToolbox.getOrCreatePositionData(endEffector, controllerIndex, DESIRED, isEnabled, true);
      yoCurrentPosition = fbToolbox.getOrCreatePositionData(endEffector, controllerIndex, CURRENT, isEnabled, true);
      yoErrorPosition = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ERROR, POSITION, isEnabled, false);

      if (computeIntegralTerm)
         yoErrorPositionIntegrated = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ERROR_INTEGRATED, POSITION, isEnabled, false);
      else
         yoErrorPositionIntegrated = null;

      yoDesiredLinearVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, LINEAR_VELOCITY, isEnabled, true);

      if (ccToolbox.isEnableInverseDynamicsModule() || ccToolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentLinearVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, CURRENT, LINEAR_VELOCITY, isEnabled, true);
         yoErrorLinearVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ERROR, LINEAR_VELOCITY, isEnabled, false);
         DoubleProvider breakFrequency = fbToolbox.getErrorVelocityFilterBreakFrequency(endEffectorName);
         if (breakFrequency != null)
         {
            yoFilteredErrorLinearVelocity = fbToolbox.getOrCreateAlphaFilteredVectorData3D(endEffector,
                                                                                           controllerIndex,
                                                                                           ERROR,
                                                                                           LINEAR_VELOCITY,
                                                                                           dt,
                                                                                           breakFrequency,
                                                                                           isEnabled,
                                                                                           false);
         }
         else
         {
            yoFilteredErrorLinearVelocity = null;
         }

         if (ccToolbox.isEnableInverseDynamicsModule())
         {
            yoDesiredLinearAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, LINEAR_ACCELERATION, isEnabled, true);
            yoFeedForwardLinearAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector,
                                                                                controllerIndex,
                                                                                FEEDFORWARD,
                                                                                LINEAR_ACCELERATION,
                                                                                isEnabled,
                                                                                false);
            yoFeedbackLinearAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector,
                                                                             controllerIndex,
                                                                             Type.FEEDBACK,
                                                                             LINEAR_ACCELERATION,
                                                                             isEnabled,
                                                                             false);
            rateLimitedFeedbackLinearAcceleration = fbToolbox.getOrCreateRateLimitedVectorData3D(endEffector,
                                                                                                 controllerIndex,
                                                                                                 FEEDBACK,
                                                                                                 LINEAR_ACCELERATION,
                                                                                                 dt,
                                                                                                 maximumRate,
                                                                                                 isEnabled,
                                                                                                 false);
            yoAchievedLinearAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, Type.ACHIEVED, LINEAR_ACCELERATION, isEnabled, true);
         }
         else
         {
            yoDesiredLinearAcceleration = null;
            yoFeedForwardLinearAcceleration = null;
            yoFeedbackLinearAcceleration = null;
            rateLimitedFeedbackLinearAcceleration = null;
            yoAchievedLinearAcceleration = null;
         }

         if (ccToolbox.isEnableVirtualModelControlModule())
         {
            yoDesiredLinearForce = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, LINEAR_FORCE, isEnabled, true);
            yoFeedForwardLinearForce = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDFORWARD, LINEAR_FORCE, isEnabled, false);
            yoFeedbackLinearForce = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, Type.FEEDBACK, LINEAR_FORCE, isEnabled, false);
            rateLimitedFeedbackLinearForce = fbToolbox.getOrCreateRateLimitedVectorData3D(endEffector,
                                                                                          controllerIndex,
                                                                                          FEEDBACK,
                                                                                          LINEAR_FORCE,
                                                                                          dt,
                                                                                          maximumRate,
                                                                                          isEnabled,
                                                                                          false);
         }
         else
         {
            yoDesiredLinearForce = null;
            yoFeedForwardLinearForce = null;
            yoFeedbackLinearForce = null;
            rateLimitedFeedbackLinearForce = null;
         }
      }
      else
      {
         yoCurrentLinearVelocity = null;
         yoErrorLinearVelocity = null;
         yoFilteredErrorLinearVelocity = null;

         yoDesiredLinearAcceleration = null;
         yoFeedForwardLinearAcceleration = null;
         yoFeedbackLinearAcceleration = null;
         rateLimitedFeedbackLinearAcceleration = null;
         yoAchievedLinearAcceleration = null;

         yoDesiredLinearForce = null;
         yoFeedForwardLinearForce = null;
         yoFeedbackLinearForce = null;
         rateLimitedFeedbackLinearForce = null;
      }

      if (ccToolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackLinearVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDBACK, LINEAR_VELOCITY, isEnabled, false);
         yoFeedForwardLinearVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDFORWARD, LINEAR_VELOCITY, isEnabled, false);
         rateLimitedFeedbackLinearVelocity = fbToolbox.getOrCreateRateLimitedVectorData3D(endEffector,
                                                                                          controllerIndex,
                                                                                          FEEDBACK,
                                                                                          LINEAR_VELOCITY,
                                                                                          dt,
                                                                                          maximumRate,
                                                                                          isEnabled,
                                                                                          false);
      }
      else
      {
         yoFeedbackLinearVelocity = null;
         yoFeedForwardLinearVelocity = null;
         rateLimitedFeedbackLinearVelocity = null;
      }
   }

   public void submitFeedbackControlCommand(PointFeedbackControlCommand command)
   {
      if (command.getEndEffector() != endEffector)
         throw new FeedbackControllerException("Wrong end effector - received: " + command.getEndEffector() + ", expected: " + endEffector);

      currentCommandId = command.getCommandId();
      base = command.getBase();
      controlBaseFrame = command.getControlBaseFrame();

      inverseDynamicsOutput.set(command.getSpatialAccelerationCommand());

      gains.set(command.getGains());
      command.getSpatialAccelerationCommand().getSelectionMatrix(selectionMatrix);
      linearGainsFrame = command.getLinearGainsFrame();

      command.getBodyFixedPointIncludingFrame(desiredPosition);
      controlFrame.setTranslationToParent(desiredPosition);

      yoDesiredPosition.setIncludingFrame(command.getReferencePosition());
      yoDesiredPosition.setCommandId(currentCommandId);
      yoDesiredLinearVelocity.setIncludingFrame(command.getReferenceLinearVelocity());
      yoDesiredLinearVelocity.checkReferenceFrameMatch(yoDesiredPosition);
      yoDesiredLinearVelocity.setCommandId(currentCommandId);
      if (yoFeedForwardLinearVelocity != null)
      {
         yoFeedForwardLinearVelocity.setIncludingFrame(command.getReferenceLinearVelocity());
         yoFeedForwardLinearVelocity.checkReferenceFrameMatch(yoDesiredPosition);
         yoFeedForwardLinearVelocity.setCommandId(currentCommandId);
      }
      if (yoFeedForwardLinearAcceleration != null)
      {
         yoFeedForwardLinearAcceleration.setIncludingFrame(command.getReferenceLinearAcceleration());
         yoFeedForwardLinearAcceleration.checkReferenceFrameMatch(yoDesiredPosition);
         yoFeedForwardLinearAcceleration.setCommandId(currentCommandId);
      }
      if (yoFeedForwardLinearForce != null)
      {
         yoFeedForwardLinearForce.setIncludingFrame(command.getReferenceForce());
         yoFeedForwardLinearForce.checkReferenceFrameMatch(yoDesiredPosition);
         yoFeedForwardLinearForce.setCommandId(currentCommandId);
      }
   }

   @Override
   public void setEnabled(boolean isEnabled)
   {
      this.isEnabled.set(isEnabled);
   }

   @Override
   public void initialize()
   { // TODO: See SpatialFeedbackController.initialize()
      if (rateLimitedFeedbackLinearAcceleration != null)
         rateLimitedFeedbackLinearAcceleration.reset();
      if (rateLimitedFeedbackLinearVelocity != null)
         rateLimitedFeedbackLinearVelocity.reset();
      if (yoFilteredErrorLinearVelocity != null)
         yoFilteredErrorLinearVelocity.reset();
      if (yoErrorPositionIntegrated != null)
         yoErrorPositionIntegrated.setToZero(worldFrame);
   }

   private final FrameVector3D proportionalFeedback = new FrameVector3D();
   private final FrameVector3D derivativeFeedback = new FrameVector3D();
   private final FrameVector3D integralFeedback = new FrameVector3D();

   @Override
   public void computeInverseDynamics()
   {
      if (!isEnabled())
         return;

      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);
      feedForwardLinearAcceleration.setIncludingFrame(yoFeedForwardLinearAcceleration);
      feedForwardLinearAcceleration.changeFrame(controlFrame);

      desiredLinearAcceleration.setIncludingFrame(proportionalFeedback);
      desiredLinearAcceleration.add(derivativeFeedback);
      desiredLinearAcceleration.add(integralFeedback);
      desiredLinearAcceleration.clipToMaxNorm(gains.getMaximumFeedback());
      yoFeedbackLinearAcceleration.setIncludingFrame(desiredLinearAcceleration);
      yoFeedbackLinearAcceleration.changeFrame(trajectoryFrame);
      yoFeedbackLinearAcceleration.setCommandId(currentCommandId);
      rateLimitedFeedbackLinearAcceleration.changeFrame(trajectoryFrame);
      rateLimitedFeedbackLinearAcceleration.update();
      rateLimitedFeedbackLinearAcceleration.setCommandId(currentCommandId);
      desiredLinearAcceleration.setIncludingFrame(rateLimitedFeedbackLinearAcceleration);

      desiredLinearAcceleration.changeFrame(controlFrame);
      desiredLinearAcceleration.add(feedForwardLinearAcceleration);

      yoDesiredLinearAcceleration.setIncludingFrame(desiredLinearAcceleration);
      yoDesiredLinearAcceleration.changeFrame(trajectoryFrame);
      yoDesiredLinearAcceleration.setCommandId(currentCommandId);

      addCoriolisAcceleration(desiredLinearAcceleration);

      inverseDynamicsOutput.setLinearAcceleration(controlFrame, desiredLinearAcceleration);
   }

   @Override
   public void computeInverseKinematics()
   {
      if (!isEnabled())
         return;

      inverseKinematicsOutput.setProperties(inverseDynamicsOutput);
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      feedForwardLinearVelocity.setIncludingFrame(yoFeedForwardLinearVelocity);
      computeProportionalTerm(proportionalFeedback);
      computeIntegralTerm(integralFeedback);

      desiredLinearVelocity.setIncludingFrame(proportionalFeedback);
      desiredLinearVelocity.add(integralFeedback);
      desiredLinearVelocity.clipToMaxNorm(gains.getMaximumFeedback());
      yoFeedbackLinearVelocity.setIncludingFrame(desiredLinearVelocity);
      yoFeedbackLinearVelocity.changeFrame(trajectoryFrame);
      yoFeedbackLinearVelocity.setCommandId(currentCommandId);
      // If the trajectory frame changed reset the rate limited variable
      if (rateLimitedFeedbackLinearVelocity.getReferenceFrame() != trajectoryFrame)
      {
         rateLimitedFeedbackLinearVelocity.setReferenceFrame(trajectoryFrame);
         rateLimitedFeedbackLinearVelocity.reset();
      }
      rateLimitedFeedbackLinearVelocity.update();
      rateLimitedFeedbackLinearVelocity.setCommandId(currentCommandId);
      desiredLinearVelocity.setIncludingFrame(rateLimitedFeedbackLinearVelocity);

      desiredLinearVelocity.add(feedForwardLinearVelocity);

      yoDesiredLinearVelocity.setIncludingFrame(desiredLinearVelocity);
      yoDesiredLinearVelocity.changeFrame(trajectoryFrame);
      yoDesiredLinearVelocity.setCommandId(currentCommandId);

      desiredLinearVelocity.changeFrame(controlFrame);
      inverseKinematicsOutput.setLinearVelocity(controlFrame, desiredLinearVelocity);
   }

   @Override
   public void computeVirtualModelControl()
   {
      if (!isEnabled())
         return;

      computeFeedbackForce();

      if (isRootBody)
      {
         desiredLinearForce.changeFrame(worldFrame);

         virtualModelControlRootOutput.setProperties(inverseDynamicsOutput);
         virtualModelControlRootOutput.setLinearMomentumRate(desiredLinearForce);
      }
      else
      {
         virtualModelControlOutput.setProperties(inverseDynamicsOutput);
         virtualModelControlOutput.setLinearForce(controlFrame, desiredLinearForce);
      }
   }

   private void computeFeedbackForce()
   {
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      feedForwardLinearForce.setIncludingFrame(yoFeedForwardLinearForce);
      feedForwardLinearForce.changeFrame(controlFrame);

      computeProportionalTerm(proportionalFeedback);
      computeDerivativeTerm(derivativeFeedback);
      computeIntegralTerm(integralFeedback);

      desiredLinearForce.setIncludingFrame(proportionalFeedback);
      desiredLinearForce.add(derivativeFeedback);
      desiredLinearForce.add(integralFeedback);
      desiredLinearForce.clipToMaxNorm(gains.getMaximumFeedback());
      yoFeedbackLinearForce.setIncludingFrame(desiredLinearForce);
      yoFeedbackLinearForce.changeFrame(trajectoryFrame);
      yoFeedbackLinearForce.setCommandId(currentCommandId);
      rateLimitedFeedbackLinearForce.changeFrame(trajectoryFrame);
      rateLimitedFeedbackLinearForce.update();
      rateLimitedFeedbackLinearForce.setCommandId(currentCommandId);
      desiredLinearForce.setIncludingFrame(rateLimitedFeedbackLinearForce);

      desiredLinearForce.changeFrame(controlFrame);
      desiredLinearForce.add(feedForwardLinearForce);

      yoDesiredLinearForce.setIncludingFrame(desiredLinearForce);
      yoDesiredLinearForce.changeFrame(trajectoryFrame);
      yoDesiredLinearForce.setCommandId(currentCommandId);
   }

   private final SpatialAcceleration achievedSpatialAccelerationVector = new SpatialAcceleration();

   @Override
   public void computeAchievedAcceleration()
   {
      achievedSpatialAccelerationVector.setIncludingFrame(rigidBodyAccelerationProvider.getRelativeAcceleration(base, endEffector));
      achievedSpatialAccelerationVector.changeFrame(controlFrame);
      achievedLinearAcceleration.setIncludingFrame(achievedSpatialAccelerationVector.getLinearPart());
      subtractCoriolisAcceleration(achievedLinearAcceleration);
      yoAchievedLinearAcceleration.setIncludingFrame(achievedLinearAcceleration);
      yoAchievedLinearAcceleration.changeFrame(yoDesiredPosition.getReferenceFrame());
      yoAchievedLinearAcceleration.setCommandId(currentCommandId);
   }

   /**
    * Computes the feedback term resulting from the error in position:<br>
    * x<sub>FB</sub> = kp * (x<sub>desired</sub> - x<sub>current</sub>)
    * <p>
    * The desired position of the {@code controlFrame} is obtained from {@link #yoDesiredPosition}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentPosition} and {@link #yoErrorPosition}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   private void computeProportionalTerm(FrameVector3D feedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      yoCurrentPosition.setToZero(controlFrame);
      yoCurrentPosition.changeFrame(trajectoryFrame);
      yoCurrentPosition.setCommandId(currentCommandId);

      desiredPosition.setIncludingFrame(yoDesiredPosition);
      desiredPosition.changeFrame(controlFrame);

      feedbackTermToPack.setIncludingFrame(desiredPosition);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxNorm(gains.getMaximumProportionalError());

      yoErrorPosition.setIncludingFrame(feedbackTermToPack);
      yoErrorPosition.changeFrame(trajectoryFrame);
      yoErrorPosition.setCommandId(currentCommandId);

      if (linearGainsFrame != null)
         feedbackTermToPack.changeFrame(linearGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      gains.getProportionalGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);

      feedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the error in linear velocity:<br>
    * x<sub>FB</sub> = kd * (xDot<sub>desired</sub> - xDot<sub>current</sub>)
    * <p>
    * The desired linear velocity of the {@code controlFrame}'s origin relative to the {@code base} is
    * obtained from {@link #yoDesiredLinearVelocity}.
    * </p>
    * <p>
    * This method also updates {@link #yoCurrentLinearVelocity} and {@link #yoErrorLinearVelocity}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified
    */
   private void computeDerivativeTerm(FrameVector3D feedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      controlFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      yoCurrentLinearVelocity.setIncludingFrame(currentTwist.getLinearPart());
      yoCurrentLinearVelocity.changeFrame(trajectoryFrame);
      yoCurrentLinearVelocity.setCommandId(currentCommandId);

      feedbackTermToPack.setToZero(trajectoryFrame);
      feedbackTermToPack.sub(yoDesiredLinearVelocity, yoCurrentLinearVelocity);
      feedbackTermToPack.changeFrame(controlFrame);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxNorm(gains.getMaximumDerivativeError());

      if (yoFilteredErrorLinearVelocity != null)
      {
         // If the trajectory frame changed reset the filter.
         if (yoFilteredErrorLinearVelocity.getReferenceFrame() != trajectoryFrame)
         {
            yoFilteredErrorLinearVelocity.setReferenceFrame(trajectoryFrame);
            yoFilteredErrorLinearVelocity.reset();
         }
         feedbackTermToPack.changeFrame(trajectoryFrame);
         yoErrorLinearVelocity.setIncludingFrame(feedbackTermToPack);
         yoFilteredErrorLinearVelocity.update();
         yoFilteredErrorLinearVelocity.setCommandId(currentCommandId);
         feedbackTermToPack.set(yoFilteredErrorLinearVelocity);
      }
      else
      {
         yoErrorLinearVelocity.setIncludingFrame(feedbackTermToPack);
      }
      yoErrorLinearVelocity.changeFrame(trajectoryFrame);
      yoErrorLinearVelocity.setCommandId(currentCommandId);

      if (linearGainsFrame != null)
         feedbackTermToPack.changeFrame(linearGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      gains.getDerivativeGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);

      feedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes the feedback term resulting from the integrated error in position:<br>
    * x<sub>FB</sub> = ki * &int;<sup>t</sup> (x<sub>desired</sub> - x<sub>current</sub>)
    * <p>
    * The current error in position of the {@code controlFrame} is obtained from
    * {@link #yoErrorPosition}.
    * </p>
    * <p>
    * This method also updates {@link #yoErrorPositionIntegrated}.
    * </p>
    *
    * @param feedbackTermToPack the value of the feedback term x<sub>FB</sub>. Modified.
    */
   private void computeIntegralTerm(FrameVector3D feedbackTermToPack)
   {
      if (!computeIntegralTerm)
      {
         feedbackTermToPack.setToZero(controlFrame);
         return;
      }
      double maximumIntegralError = gains.getMaximumIntegralError();
      ReferenceFrame trajectoryFrame = yoDesiredPosition.getReferenceFrame();

      if (maximumIntegralError < 1.0e-5)
      {
         feedbackTermToPack.setToZero(controlFrame);
         yoErrorPositionIntegrated.setToZero(trajectoryFrame);
         yoErrorPositionIntegrated.setCommandId(currentCommandId);
         return;
      }

      // If the trajectory frame changed reset the integration.
      if (yoErrorPositionIntegrated.getReferenceFrame() != trajectoryFrame)
      {
         yoErrorPositionIntegrated.setToZero(trajectoryFrame);
      }

      feedbackTermToPack.setIncludingFrame(yoErrorPosition);
      feedbackTermToPack.scale(dt);
      feedbackTermToPack.add(yoErrorPositionIntegrated);
      feedbackTermToPack.changeFrame(controlFrame);
      selectionMatrix.applyLinearSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxNorm(maximumIntegralError);
      yoErrorPositionIntegrated.setIncludingFrame(feedbackTermToPack);
      yoErrorPositionIntegrated.changeFrame(trajectoryFrame);
      yoErrorPositionIntegrated.setCommandId(currentCommandId);

      if (linearGainsFrame != null)
         feedbackTermToPack.changeFrame(linearGainsFrame);
      else
         feedbackTermToPack.changeFrame(controlFrame);

      gains.getIntegralGainMatrix(tempGainMatrix);
      tempGainMatrix.transform(feedbackTermToPack);

      feedbackTermToPack.changeFrame(controlFrame);
   }

   /**
    * Computes and adds the bias acceleration resulting from the combination of the current linear and
    * angular velocity of the control frame.
    * <p>
    * This is needed when going from a linear acceleration expressed in an inertial frame to a moving
    * frame attached to the end-effector.
    * </p>
    * <p>
    * Intuitively, the Coriolis acceleration only appears when measuring the acceleration from a moving
    * frame, here a frame attache to the end-effector.
    * </p>
    *
    * @param linearAccelerationToModify the linear acceleration vector to which the bias is to be
    *                                   subtracted. Its frame is changed to {@code controlFrame}.
    *                                   Modified.
    */
   private void addCoriolisAcceleration(FrameVector3D linearAccelerationToModify)
   {
      controlFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      currentAngularVelocity.setIncludingFrame(currentTwist.getAngularPart());
      currentLinearVelocity.setIncludingFrame(currentTwist.getLinearPart());

      biasLinearAcceleration.setToZero(controlFrame);
      biasLinearAcceleration.cross(currentLinearVelocity, currentAngularVelocity);
      linearAccelerationToModify.changeFrame(controlFrame);
      linearAccelerationToModify.add(biasLinearAcceleration);
   }

   /**
    * Computes and subtracts the bias acceleration resulting from the combination of the current linear
    * and angular velocity of the control frame.
    * <p>
    * This is needed when going from a linear acceleration expressed in a moving frame attached to the
    * end-effector to an inertial frame.
    * </p>
    * <p>
    * Intuitively, the Coriolis acceleration only appears when measuring the acceleration from a moving
    * frame, here a frame attache to the end-effector.
    * </p>
    *
    * @param linearAccelerationToModify the linear acceleration vector to which the bias is to be
    *                                   added. Its frame is changed to {@code worldFrame}. Modified.
    */
   private void subtractCoriolisAcceleration(FrameVector3D linearAccelerationToModify)
   {
      controlFrame.getTwistRelativeToOther(controlBaseFrame, currentTwist);
      currentAngularVelocity.setIncludingFrame(currentTwist.getAngularPart());
      currentLinearVelocity.setIncludingFrame(currentTwist.getLinearPart());

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
