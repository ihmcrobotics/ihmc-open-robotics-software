package us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.taskspace;

import static us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox.appendIndex;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.ANGULAR_ACCELERATION;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.ANGULAR_TORQUE;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.ANGULAR_VELOCITY;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D.ROTATION_VECTOR;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.ACHIEVED;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.CURRENT;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.DESIRED;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.ERROR;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.ERROR_CUMULATED;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.ERROR_INTEGRATED;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.FEEDBACK;
import static us.ihmc.commonWalkingControlModules.controllerCore.data.Type.FEEDFORWARD;

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
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class OrientationFeedbackController implements FeedbackControllerInterface
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final YoRegistry registry;

   protected final YoBoolean isEnabled;

   protected final FBQuaternion3D yoDesiredOrientation;
   protected final FBQuaternion3D yoCurrentOrientation;
   protected final FBQuaternion3D yoErrorOrientation;

   protected final FBQuaternion3D yoErrorOrientationCumulated;

   protected final FBVector3D yoDesiredRotationVector;
   protected final FBVector3D yoCurrentRotationVector;
   protected final FBVector3D yoErrorRotationVector;

   protected final FBVector3D yoErrorRotationVectorIntegrated;

   protected final FBVector3D yoDesiredAngularVelocity;
   protected final FBVector3D yoCurrentAngularVelocity;
   protected final FBVector3D yoErrorAngularVelocity;
   protected final FBAlphaFilteredVector3D yoFilteredErrorAngularVelocity;
   protected final FBVector3D yoFeedForwardAngularVelocity;
   protected final FBVector3D yoFeedbackAngularVelocity;
   protected final FBRateLimitedVector3D rateLimitedFeedbackAngularVelocity;

   protected final FBVector3D yoDesiredAngularAcceleration;
   protected final FBVector3D yoFeedForwardAngularAcceleration;
   protected final FBVector3D yoFeedbackAngularAcceleration;
   protected final FBRateLimitedVector3D rateLimitedFeedbackAngularAcceleration;
   protected final FBVector3D yoAchievedAngularAcceleration;

   protected final FBVector3D yoDesiredAngularTorque;
   protected final FBVector3D yoFeedForwardAngularTorque;
   protected final FBVector3D yoFeedbackAngularTorque;
   protected final FBRateLimitedVector3D rateLimitedFeedbackAngularTorque;

   protected final FrameQuaternion desiredOrientation = new FrameQuaternion();
   protected final FrameQuaternion errorOrientationCumulated = new FrameQuaternion();

   protected final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   protected final FrameVector3D feedForwardAngularVelocity = new FrameVector3D();

   protected final FrameVector3D desiredAngularAcceleration = new FrameVector3D();
   protected final FrameVector3D feedForwardAngularAction = new FrameVector3D();
   protected final FrameVector3D feedForwardAngularAcceleration = new FrameVector3D();
   protected final FrameVector3D achievedAngularAcceleration = new FrameVector3D();

   protected final FrameVector3D desiredAngularTorque = new FrameVector3D();

   protected final Twist currentTwist = new Twist();
   protected final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   protected final SpatialAccelerationCommand inverseDynamicsOutput = new SpatialAccelerationCommand();
   protected final SpatialVelocityCommand inverseKinematicsOutput = new SpatialVelocityCommand();
   protected final VirtualTorqueCommand virtualModelControlOutput = new VirtualTorqueCommand();
   protected final MomentumRateCommand virtualModelControlRootOutput = new MomentumRateCommand();

   protected final YoPID3DGains gains;
   protected final Matrix3D tempGainMatrix = new Matrix3D();

   protected final RigidBodyAccelerationProvider rigidBodyAccelerationProvider;

   protected RigidBodyBasics base;
   protected ReferenceFrame controlBaseFrame;
   protected ReferenceFrame angularGainsFrame;

   protected final RigidBodyBasics rootBody;
   protected final RigidBodyBasics endEffector;
   protected final MovingReferenceFrame endEffectorFrame;

   protected final double dt;
   protected final boolean isRootBody;
   protected final boolean computeIntegralTerm;

   protected final int controllerIndex;
   protected int currentCommandId;

   public OrientationFeedbackController(RigidBodyBasics endEffector, WholeBodyControlCoreToolbox ccToolbox, FeedbackControllerToolbox fbToolbox,
                                        YoRegistry parentRegistry)
   {
      this(endEffector, 0, ccToolbox, fbToolbox, parentRegistry);
   }

   public OrientationFeedbackController(RigidBodyBasics endEffector, int controllerIndex, WholeBodyControlCoreToolbox ccToolbox,
                                        FeedbackControllerToolbox fbToolbox, YoRegistry parentRegistry)
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
      registry = new YoRegistry(appendIndex(endEffectorName, controllerIndex) + "OrientationFBController");
      dt = ccToolbox.getControlDT();
      gains = fbToolbox.getOrCreateOrientationGains(endEffector, controllerIndex, computeIntegralTerm);
      YoDouble maximumRate = gains.getYoMaximumFeedbackRate();

      endEffectorFrame = endEffector.getBodyFixedFrame();

      isEnabled = new YoBoolean(appendIndex(endEffectorName, controllerIndex) + "IsOrientationFBControllerEnabled", registry);
      isEnabled.set(false);

      yoDesiredOrientation = fbToolbox.getOrCreateOrientationData(endEffector, controllerIndex, DESIRED, isEnabled);
      yoCurrentOrientation = fbToolbox.getOrCreateOrientationData(endEffector, controllerIndex, CURRENT, isEnabled);
      yoErrorOrientation = fbToolbox.getOrCreateOrientationData(endEffector, controllerIndex, ERROR, isEnabled);

      yoDesiredRotationVector = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, ROTATION_VECTOR, isEnabled);
      yoCurrentRotationVector = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, CURRENT, ROTATION_VECTOR, isEnabled);
      yoErrorRotationVector = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ERROR, ROTATION_VECTOR, isEnabled);

      if (computeIntegralTerm)
      {
         yoErrorOrientationCumulated = fbToolbox.getOrCreateOrientationData(endEffector, controllerIndex, ERROR_CUMULATED, isEnabled);
         yoErrorRotationVectorIntegrated = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ERROR_INTEGRATED, ROTATION_VECTOR, isEnabled);
      }
      else
      {
         yoErrorOrientationCumulated = null;
         yoErrorRotationVectorIntegrated = null;
      }

      yoDesiredAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, ANGULAR_VELOCITY, isEnabled);

      if (ccToolbox.isEnableInverseDynamicsModule() || ccToolbox.isEnableVirtualModelControlModule())
      {
         yoCurrentAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, CURRENT, ANGULAR_VELOCITY, isEnabled);
         yoErrorAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ERROR, ANGULAR_VELOCITY, isEnabled);
         DoubleProvider breakFrequency = fbToolbox.getErrorVelocityFilterBreakFrequency(endEffectorName);
         if (breakFrequency != null)
            yoFilteredErrorAngularVelocity = fbToolbox.getOrCreateAlphaFilteredVectorData3D(endEffector,
                                                                                            controllerIndex,
                                                                                            ERROR,
                                                                                            ANGULAR_VELOCITY,
                                                                                            dt,
                                                                                            breakFrequency,
                                                                                            isEnabled);
         else
            yoFilteredErrorAngularVelocity = null;

         if (ccToolbox.isEnableInverseDynamicsModule())
         {
            yoDesiredAngularAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, ANGULAR_ACCELERATION, isEnabled);
            yoFeedForwardAngularAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDFORWARD, ANGULAR_ACCELERATION, isEnabled);
            yoFeedbackAngularAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDBACK, ANGULAR_ACCELERATION, isEnabled);
            rateLimitedFeedbackAngularAcceleration = fbToolbox.getOrCreateRateLimitedVectorData3D(endEffector,
                                                                                                  controllerIndex,
                                                                                                  FEEDBACK,
                                                                                                  ANGULAR_ACCELERATION,
                                                                                                  dt,
                                                                                                  maximumRate,
                                                                                                  isEnabled);
            yoAchievedAngularAcceleration = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, ACHIEVED, ANGULAR_ACCELERATION, isEnabled);
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
            yoDesiredAngularTorque = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, DESIRED, ANGULAR_TORQUE, isEnabled);
            yoFeedForwardAngularTorque = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDFORWARD, ANGULAR_TORQUE, isEnabled);
            yoFeedbackAngularTorque = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDBACK, ANGULAR_TORQUE, isEnabled);
            rateLimitedFeedbackAngularTorque = fbToolbox.getOrCreateRateLimitedVectorData3D(endEffector,
                                                                                            controllerIndex,
                                                                                            FEEDBACK,
                                                                                            ANGULAR_TORQUE,
                                                                                            dt,
                                                                                            maximumRate,
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

      if (ccToolbox.isEnableInverseKinematicsModule())
      {
         yoFeedbackAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDBACK, ANGULAR_VELOCITY, isEnabled);
         yoFeedForwardAngularVelocity = fbToolbox.getOrCreateVectorData3D(endEffector, controllerIndex, FEEDFORWARD, ANGULAR_ACCELERATION, isEnabled);
         rateLimitedFeedbackAngularVelocity = fbToolbox.getOrCreateRateLimitedVectorData3D(endEffector,
                                                                                           controllerIndex,
                                                                                           FEEDBACK,
                                                                                           ANGULAR_VELOCITY,
                                                                                           dt,
                                                                                           maximumRate,
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
      if (yoErrorOrientationCumulated != null)
         yoErrorOrientationCumulated.setToZero(worldFrame);
      if (yoErrorRotationVectorIntegrated != null)
         yoErrorRotationVectorIntegrated.setToZero(worldFrame);
   }

   protected final FrameVector3D proportionalFeedback = new FrameVector3D();
   protected final FrameVector3D derivativeFeedback = new FrameVector3D();
   protected final FrameVector3D integralFeedback = new FrameVector3D();

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
      yoFeedbackAngularAcceleration.setCommandId(currentCommandId);
      rateLimitedFeedbackAngularAcceleration.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularAcceleration.update();
      rateLimitedFeedbackAngularAcceleration.setCommandId(currentCommandId);
      desiredAngularAcceleration.setIncludingFrame(rateLimitedFeedbackAngularAcceleration);

      desiredAngularAcceleration.changeFrame(endEffectorFrame);
      desiredAngularAcceleration.add(feedForwardAngularAcceleration);

      yoDesiredAngularAcceleration.setIncludingFrame(desiredAngularAcceleration);
      yoDesiredAngularAcceleration.changeFrame(trajectoryFrame);
      yoDesiredAngularAcceleration.setCommandId(currentCommandId);

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
      yoFeedbackAngularVelocity.setCommandId(currentCommandId);
      rateLimitedFeedbackAngularVelocity.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularVelocity.update();
      rateLimitedFeedbackAngularVelocity.setCommandId(currentCommandId);
      desiredAngularVelocity.setIncludingFrame(rateLimitedFeedbackAngularVelocity);

      desiredAngularVelocity.add(feedForwardAngularVelocity);

      yoDesiredAngularVelocity.setIncludingFrame(desiredAngularVelocity);
      yoDesiredAngularVelocity.changeFrame(trajectoryFrame);
      yoDesiredAngularVelocity.setCommandId(currentCommandId);

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

   protected void computeFeedbackTorque()
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
      yoFeedbackAngularTorque.setCommandId(currentCommandId);
      rateLimitedFeedbackAngularTorque.changeFrame(trajectoryFrame);
      rateLimitedFeedbackAngularTorque.update();
      rateLimitedFeedbackAngularTorque.setCommandId(currentCommandId);
      desiredAngularTorque.setIncludingFrame(rateLimitedFeedbackAngularTorque);

      desiredAngularTorque.changeFrame(endEffectorFrame);
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
   protected void computeProportionalTerm(FrameVector3D feedbackTermToPack)
   {
      ReferenceFrame trajectoryFrame = yoDesiredOrientation.getReferenceFrame();

      yoCurrentOrientation.setToZero(endEffectorFrame);
      yoCurrentOrientation.changeFrame(trajectoryFrame);
      yoCurrentOrientation.setCommandId(currentCommandId);
      yoCurrentOrientation.getRotationVector(yoCurrentRotationVector);
      yoCurrentRotationVector.setCommandId(currentCommandId);

      desiredOrientation.setIncludingFrame(yoDesiredOrientation);
      desiredOrientation.changeFrame(endEffectorFrame);

      desiredOrientation.normalizeAndLimitToPi();
      desiredOrientation.getRotationVector(feedbackTermToPack);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(gains.getMaximumProportionalError());

      yoErrorRotationVector.setIncludingFrame(feedbackTermToPack);
      yoErrorRotationVector.changeFrame(trajectoryFrame);
      yoErrorRotationVector.setCommandId(currentCommandId);
      yoErrorOrientation.setRotationVectorIncludingFrame(yoErrorRotationVector);
      yoErrorRotationVector.setCommandId(currentCommandId);

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
      yoCurrentAngularVelocity.setCommandId(currentCommandId);

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
         yoFilteredErrorAngularVelocity.setCommandId(currentCommandId);
         feedbackTermToPack.set(yoFilteredErrorAngularVelocity);
      }
      else
      {
         yoErrorAngularVelocity.setIncludingFrame(feedbackTermToPack);
      }
      yoErrorAngularVelocity.changeFrame(trajectoryFrame);
      yoErrorAngularVelocity.setCommandId(currentCommandId);

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
      feedbackTermToPack.changeFrame(endEffectorFrame);
      selectionMatrix.applyAngularSelection(feedbackTermToPack);
      feedbackTermToPack.clipToMaxLength(maximumIntegralError);
      yoErrorRotationVectorIntegrated.setIncludingFrame(feedbackTermToPack);
      yoErrorRotationVectorIntegrated.changeFrame(trajectoryFrame);
      yoErrorRotationVectorIntegrated.setCommandId(currentCommandId);

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
