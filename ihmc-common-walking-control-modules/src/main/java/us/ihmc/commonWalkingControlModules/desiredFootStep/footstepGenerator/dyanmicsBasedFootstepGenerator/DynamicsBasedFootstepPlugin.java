package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.dyanmicsBasedFootstepGenerator;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingPlugin;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class DynamicsBasedFootstepPlugin implements HumanoidSteppingPlugin
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DynamicsBasedFootstepParameters parameters;

   private final String variableNameSuffix = "QFP";

   private final CommonHumanoidReferenceFrames referenceFrames;

   private final double updateDT;
   private final double gravity = 9.81;

   // Estimates
   private final YoFramePoint3D currentCoMPosition = new YoFramePoint3D("currentCoMPosition" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D currentCoMVelocity = new YoFrameVector3D("currentCoMVelocity" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);
   private final MovingReferenceFrame centerOfMassControlFrame;
   private final MovingZUpFrame centerOfMassControlZUpFrame;
   private final ReferenceFrame centerOfMassFrame;
   private final FootSoleBasedGroundPlaneEstimator groundPlaneEstimator;

   // Command/Desired output related stuff
   private final SideDependentList<DynamicsBasedFootstepTouchdownCalculator> touchdownCalculator = new SideDependentList<>();
   private final FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
   private final RecyclingArrayList<FootstepDataMessage> footsteps = footstepDataListMessage.getFootstepDataList();

   // Desired inputs
   private final YoDouble desiredTurningVelocity = new YoDouble("desiredTurningVelocity" + variableNameSuffix, registry);
   private final YoVector2D desiredVelocity = new YoVector2D("desiredVelocity" + variableNameSuffix, registry);
   private final YoBoolean ignoreWalkInputProvider = new YoBoolean("ignoreWalkInputProvider" + variableNameSuffix, registry);
   private final YoBoolean walk = new YoBoolean("walk" + variableNameSuffix, registry);
   private final YoBoolean walkPreviousValue = new YoBoolean("walkPreviousValue" + variableNameSuffix, registry);
   private final YoFrameQuaternion desiredPelvisOrientation;

   // Foot state information
   public enum FootState {SUPPORT, SWING}
   private final SideDependentList<YoEnum<FootState>> footStates = new SideDependentList<>();
   private final YoEnum<RobotSide> newestSupportSide = new YoEnum<>("newestSupportSide" + variableNameSuffix, registry, RobotSide.class);
   private final MutableObject<FootstepStatus> latestFootstepStatusReceived = new MutableObject<>(null);
   private final MutableObject<RobotSide> footstepCompletionSide = new MutableObject<>(null);

   //
   private final static Vector2DReadOnly zero2D = new Vector2D();
   private BooleanProvider walkInputProvider;
   private DoubleProvider swingHeightInputProvider;
   private DesiredVelocityProvider desiredVelocityProvider = () -> zero2D;
   private DesiredTurningVelocityProvider desiredTurningVelocityProvider = () -> 0.0;
   private DirectionalControlMessenger directionalControlMessenger;
   private StopWalkingMessenger stopWalkingMessenger;
   private StartWalkingMessenger startWalkingMessenger;
   private FootstepMessenger footstepMessenger;

   public DynamicsBasedFootstepPlugin(FullHumanoidRobotModel robotModel, CommonHumanoidReferenceFrames referenceFrames, double updateDT, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.updateDT = updateDT;
      this.referenceFrames = referenceFrames;

      parameters = new DynamicsBasedFootstepParameters(gravity, registry);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      desiredPelvisOrientation = new YoFrameQuaternion("pelvisDesiredOrientation" + variableNameSuffix, ReferenceFrame.getWorldFrame(), registry);

      centerOfMassControlFrame = new MovingReferenceFrame("centerOfMassControlFrame" + variableNameSuffix, ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
         {
            //            pelvisFrame.getTwistRelativeToOther(ReferenceFrame.getWorldFrame(), pelvisTwist);
            //            pelvisTwist.changeFrame(centerOfMassFrame); // FIXME we really want the rotation about the center of mass, relative to the world.

            twistRelativeToParentToPack.getLinearPart().setMatchingFrame(getCenterOfMassVelocity());
            //            twistRelativeToParentToPack.getAngularPart().setMatchingFrame(pelvisTwist.getAngularPart());
         }

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.getTranslation().set(currentCoMPosition);
            transformToParent.getRotation().set(desiredPelvisOrientation);
         }
      };

      centerOfMassControlZUpFrame = new MovingZUpFrame(centerOfMassControlFrame, "centerOfMassControlZUpFrame" + variableNameSuffix);

      groundPlaneEstimator = new FootSoleBasedGroundPlaneEstimator(centerOfMassControlZUpFrame, referenceFrames, yoGraphicsListRegistry, registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         footStates.put(robotSide, new YoEnum<>("footStates" + variableNameSuffix, registry, FootState.class));
         touchdownCalculator = new DynamicsBasedFootstepTouchdownCalculator(robotSide, centerOfMassControlZUpFrame, getCenterOfMassVelocity(), , robotModel, groundPlaneEstimator, parameters, gravity, desiredVelocityProvider, registry);
      }
   }

   @Override
   public void update(double time)
   {
      groundPlaneEstimator.update();

      currentCoMPosition.setFromReferenceFrame(centerOfMassFrame);

      handleDesiredsFromProviders();

      // TODO maybe wrong?
      desiredPelvisOrientation.setToYawOrientation(referenceFrames.getChestFrame().getTransformToWorldFrame().getRotation().getYaw());
      desiredPelvisOrientation.appendYawRotation(desiredTurningVelocity.getDoubleValue() * updateDT);

      centerOfMassControlFrame.update();
      centerOfMassControlZUpFrame.update();

      currentCoMVelocity.setMatchingFrame(controllerToolbox.getCenterOfMassVelocity());

      footstepDataListMessage.setDefaultSwingDuration(parameters.getSwingDuration());
      footstepDataListMessage.setDefaultTransferDuration(parameters.getTransferDuration());
      footstepDataListMessage.setFinalTransferDuration(parameters.getTransferDuration());
      footstepDataListMessage.setAreFootstepsAdjustable(parameters.getStepsAreAdjustable());
      footstepDataListMessage.setOffsetFootstepsWithExecutionError(parameters.getShiftUpcomingStepsWithTouchdown());

      if (!ignoreWalkInputProvider.getBooleanValue() && walkInputProvider != null)
         walk.set(walkInputProvider.getValue());

      if (!walk.getValue())
      {
         footsteps.clear();

         if (stopWalkingMessenger != null && walk.getValue() != walkPreviousValue.getValue())
         {
            stopWalkingMessenger.submitStopWalkingRequest();
         }

         walkPreviousValue.set(false);
         return;
      }
      else if (startWalkingMessenger != null && walk.getValue() != walkPreviousValue.getValue())
      {
         startWalkingMessenger.submitStartWalkingRequest();
      }

      if (walk.getValue() != walkPreviousValue.getValue())
      {
         counter = numberOfTicksBeforeSubmittingFootsteps.getValue(); // To make footsteps being sent right away.
      }

      { // Processing footstep status
         FootstepStatus statusToProcess = latestFootstepStatusReceived.getValue();

         if (statusToProcess != null)
         {
            if (statusToProcess == FootstepStatus.STARTED)
            {
               if (!footsteps.isEmpty())
                  footsteps.remove(0);
            }
            else if (statusToProcess == FootstepStatus.COMPLETED)
            {
               currentSupportSide.set(footstepCompletionSide.getValue());
               if (parameters.getNumberOfFixedFootsteps() == 0)
                  footsteps.clear();
            }
         }

         latestFootstepStatusReceived.setValue(null);
         footstepCompletionSide.setValue(null);
      }

      // Submit footsteps
      if (walk.getValue() && footstepMessenger != null)
      {
         if (counter >= numberOfTicksBeforeSubmittingFootsteps.getValue())
         {
            currentFootstepDataListCommandID.increment();
            footstepDataListMessage.setSequenceId(currentFootstepDataListCommandID.getValue());
            footstepDataListMessage.getQueueingProperties().setSequenceId(currentFootstepDataListCommandID.getValue());
            footstepDataListMessage.getQueueingProperties().setMessageId(currentFootstepDataListCommandID.getValue());
            footstepMessenger.submitFootsteps(footstepDataListMessage);
            counter = 0;
         }
         else
         {
            counter++;
         }
      }

      walkPreviousValue.set(walk.getValue());
   }

   private void handleDesiredsFromProviders()
   {
      Vector2DReadOnly desiredVelocity = desiredVelocityProvider.getDesiredVelocity();
      double desiredVelocityX = desiredVelocity.getX();
      double desiredVelocityY = desiredVelocity.getY();
      double turningVelocity = desiredTurningVelocityProvider.getTurningVelocity();

      if (desiredVelocityProvider.isUnitVelocity())
      {
         double minMaxVelocityX = maxStepLength / stepTime.getValue();
         double minMaxVelocityY = maxStepWidth / stepTime.getValue();
         desiredVelocityX = minMaxVelocityX * MathTools.clamp(desiredVelocityX, 1.0);
         desiredVelocityY = minMaxVelocityY * MathTools.clamp(desiredVelocityY, 1.0);
      }

      if (desiredTurningVelocityProvider.isUnitVelocity())
      {
         double minMaxVelocityTurn = (turnMaxAngleOutward - turnMaxAngleInward) / stepTime.getValue();
         turningVelocity = minMaxVelocityTurn * MathTools.clamp(turningVelocity, 1.0);
      }

      this.desiredVelocity.set(desiredVelocityX, desiredVelocityY);
      this.desiredTurningVelocity.set(turningVelocity);
   }

   /**
    * Sets the protocol for sending footsteps to the controller.
    *
    * @param footstepMessenger the callback used to send footsteps.
    */
   public void setFootstepMessenger(FootstepMessenger footstepMessenger)
   {
      this.footstepMessenger = footstepMessenger;
   }

   /**
    * Attaches a listener for {@code FootstepStatusMessage} to the manager.
    * <p>
    * This listener will automatically call {@link #notifyFootstepStarted(RobotSide)} and
    * {@link #notifyFootstepCompleted(RobotSide)}.
    * </p>
    *
    * @param statusMessageOutputManager the output API of the controller.
    */
   public void setFootstepStatusListener(StatusMessageOutputManager statusMessageOutputManager)
   {
      statusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, this::consumeFootstepStatus);
   }

   /**
    * Consumes a newly received message and calls {@link #notifyFootstepStarted(RobotSide)} or
    * {@link #notifyFootstepCompleted(RobotSide)} according to the status.
    *
    * @param statusMessage the newly received footstep status.
    */
   public void consumeFootstepStatus(FootstepStatusMessage statusMessage)
   {
      FootstepStatus status = FootstepStatus.fromByte(statusMessage.getFootstepStatus());
      if (status == FootstepStatus.COMPLETED)
         notifyFootstepCompleted(RobotSide.fromByte(statusMessage.getRobotSide()));
      else if (status == FootstepStatus.STARTED)
         notifyFootstepStarted(RobotSide.fromByte(statusMessage.getRobotSide()));
   }

   /**
    * Notifies this generator that a footstep has been completed.
    * <p>
    * It is used internally to keep track of the support foot from which footsteps should be generated.
    * </p>
    *
    * @param robotSide the side corresponding to the foot that just completed a footstep.
    */
   public void notifyFootstepCompleted(RobotSide robotSide)
   {
      latestFootstepStatusReceived.setValue(FootstepStatus.COMPLETED);
      footstepCompletionSide.setValue(robotSide);
      footStates.get(robotSide).set(FootState.SUPPORT);
      newestSupportSide.set(robotSide);
   }

   /**
    * Notifies this generator that a footstep has been started, i.e. the foot started swinging.
    * <p>
    * It is used internally to mark the first generated footstep as unmodifiable so it does not change
    * while the swing foot is targeting it.
    * </p>
    */
   public void notifyFootstepStarted(RobotSide robotSide)
   {
      latestFootstepStatusReceived.setValue(FootstepStatus.STARTED);
      footstepCompletionSide.setValue(null);
      footStates.get(robotSide).set(FootState.SWING);
   }

   /**
    * Sets a provider that is to be used to update the state of {@link #walk} internally on each call
    * to {@link #update(double)}.
    *
    * @param walkInputProvider the provider used to determine whether to walk or not walk.
    */
   public void setWalkInputProvider(BooleanProvider walkInputProvider)
   {
      this.walkInputProvider = walkInputProvider;
   }

   /**
    * Sets a provider that is to be used to update the desired swing height of each foot internally
    * on each call to {@link #update(double)}
    *
    * @param swingHeightInputProvider the provider used to set the swing height
    */
   public void setSwingHeightInputProvider(DoubleProvider swingHeightInputProvider)
   {
      this.swingHeightInputProvider = swingHeightInputProvider;
   }

   /**
    * Sets the provider to use for getting every tick the desired turning velocity.
    *
    * @param desiredTurningVelocityProvider provider for obtaining the desired turning velocity.
    */
   public void setDesiredTurningVelocityProvider(DesiredTurningVelocityProvider desiredTurningVelocityProvider)
   {
      this.desiredTurningVelocityProvider = desiredTurningVelocityProvider;
   }

   /**
    * Sets the provider to use for getting every tick the desired forward/lateral velocity.
    *
    * @param desiredVelocityProvider provider for obtaining the desired forward/lateral velocity.
    */
   public void setDesiredVelocityProvider(DesiredVelocityProvider desiredVelocityProvider)
   {
      this.desiredVelocityProvider = desiredVelocityProvider;
   }

   /**
    * Sets the protocol for stop walking requests to the controller.
    *
    * @param stopWalkingMessenger the callback used to send requests.
    */
   public void setStopWalkingMessenger(StopWalkingMessenger stopWalkingMessenger)
   {
      this.stopWalkingMessenger = stopWalkingMessenger;
   }

   /**
    * Sets the protocol for start walking requests to the controller.
    *
    * @param startWalkingMessenger the callback used to send requests.
    */
   public void setStartWalkingMessenger(StartWalkingMessenger startWalkingMessenger)
   {
      this.startWalkingMessenger = startWalkingMessenger;
   }

   @Override
   public void setFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {

   }

   private FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      return currentCoMVelocity;
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }
}