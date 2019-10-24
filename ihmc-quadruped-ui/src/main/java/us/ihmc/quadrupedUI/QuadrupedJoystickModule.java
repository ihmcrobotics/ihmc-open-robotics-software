package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.*;
import javafx.animation.AnimationTimer;
import net.java.games.input.Event;
import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.quadrupedUI.QuadrupedXBoxBindings.*;

public class QuadrupedJoystickModule extends AnimationTimer implements JoystickEventListener
{
   private static final int pollRateMillis = 50;
   private static final double maximumBodyHeightOffset = 0.1;
   private static final double bodyHeightDeltaPerClick = 0.01;
   private static final double endPhaseDeltaPerClick = 90.0;
   private static final double maxBodyYaw = 0.25;
   private static final double maxBodyRoll = 0.15;
   private static final double maxBodyPitch = 0.15;
   private static final double bodyOrientationShiftTime = 0.1;
   private static final double maxTranslationX = 0.25;
   private static final double maxTranslationY = 0.15;

   private final MutableDouble maxVelocityY = new MutableDouble();
   private final MutableDouble maxVelocityYaw = new MutableDouble();
   private final MutableDouble bodyHeightOffset = new MutableDouble();

   private final Messager messager;
   private final double nominalBodyHeight;
   private final Map<XBoxOneMapping, ChannelData> channelDataMap = new HashMap<>();
   private final QuadrupedXGaitSettingsBasics xGaitSettings;
   private final QuadrupedReferenceFrames referenceFrames;

   private final AtomicReference<QuadrupedSteppingStateEnum> currentSteppingState;
   private final AtomicReference<Boolean> joystickEnabled;
   private final AtomicReference<Boolean> stepTeleopEnabled;
   private final AtomicReference<Boolean> heightTeleopEnabled;
   private final AtomicReference<Boolean> bodyPoseTeleopEnabled;

   private final AtomicBoolean resetBodyPose = new AtomicBoolean();

   public QuadrupedJoystickModule(Messager messager, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, FullQuadrupedRobotModel robotModel,
                                  double nominalBodyHeight, Joystick joystick)
   {
      this.messager = messager;
      this.xGaitSettings = new QuadrupedXGaitSettings(defaultXGaitSettings);
      this.nominalBodyHeight = nominalBodyHeight;
      this.referenceFrames = new QuadrupedReferenceFrames(robotModel);

      joystick.addJoystickEventListener(this);
      joystick.setPollInterval(pollRateMillis);
      configureJoystickFilters(joystick);

      for (XBoxOneMapping channel : XBoxOneMapping.values)
      {
         channelDataMap.put(channel, new ChannelData());
      }

      channelDataMap.get(XBoxOneMapping.LEFT_TRIGGER).initialize(-1.0);
      channelDataMap.get(XBoxOneMapping.RIGHT_TRIGGER).initialize(-1.0);

      joystickEnabled = messager.createInput(QuadrupedUIMessagerAPI.EnableJoystickTopic, false);
      stepTeleopEnabled = messager.createInput(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, false);
      heightTeleopEnabled = messager.createInput(QuadrupedUIMessagerAPI.EnableHeightTeleopTopic, false);
      bodyPoseTeleopEnabled = messager.createInput(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, false);

      messager.registerTopicListener(QuadrupedUIMessagerAPI.XGaitSettingsTopic, xGaitSettings::set);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.CurrentControllerNameTopic, state ->
      {
         if (state != HighLevelControllerName.WALKING)
         {
            messager.submitMessage(QuadrupedUIMessagerAPI.EnableJoystickTopic, false);
         }
      });

      currentSteppingState = messager.createInput(QuadrupedUIMessagerAPI.CurrentSteppingStateNameTopic, null);
   }

   private static void configureJoystickFilters(Joystick device)
   {
      device.setCustomizationFilter(new JoystickCustomizationFilter(xVelocityMapping, xVelocityInvert, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(yVelocityMapping, yVelocityInvert, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(leftTurnMapping, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(rightTurnMapping, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(negativeYawMapping, negativeYawInvert, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(positiveYawMapping, positiveYawInvert, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(rollMapping, rollInvert, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(pitchMapping, pitchInvert, 0.1, 1));
   }

   @Override
   public void handle(long now)
   {
      processEnableCommand();

      if (!joystickEnabled.get())
      {
         return;
      }

      processResetBodyCommand();
      processPauseWalkingCommand();

      referenceFrames.updateFrames();

      if (resetBodyPose.getAndSet(false))
      {
         sendResetCommands();
      }

      processBodyHeightCommands();
      processBodyOrientationCommands();

      processXGaitSettingCommands();
      processStepCommands();
   }

   private void processEnableCommand()
   {
      ChannelData enableChannel = channelDataMap.get(XBoxOneMapping.START);
      if (enableChannel.hasNewData() && enableChannel.getValue() > 0.5)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableJoystickTopic, !joystickEnabled.get());
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, false);
      }
   }

   private void processResetBodyCommand()
   {
      ChannelData resetChannel = channelDataMap.get(XBoxOneMapping.SELECT);
      if (resetChannel.hasNewData() && resetChannel.getValue() > 0.5)
      {
         resetBodyPose.set(true);
      }
   }

   private void processPauseWalkingCommand()
   {
      ChannelData pauseWalkingChannel = channelDataMap.get(XBoxOneMapping.SELECT);
      if (pauseWalkingChannel.hasNewData() && pauseWalkingChannel.getValue() > 0.5)
      {
         boolean walking = currentSteppingState.get() != null && currentSteppingState.get() == QuadrupedSteppingStateEnum.STEP;
         messager.submitMessage(QuadrupedUIMessagerAPI.PauseWalkingTopic, walking);
      }
   }

   private void sendResetCommands()
   {
      bodyHeightOffset.setValue(0.0);
      sendBodyHeightSetpoint();

      sendOrientationSetpoint(0.0, 0.0, 0.0);
   }

   private void processBodyHeightCommands()
   {
      ChannelData enableHeightChannel = channelDataMap.get(enableHeightTeleop);
      if (enableHeightChannel.hasNewData() && enableHeightChannel.getValue() > 0.5)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableHeightTeleopTopic, !heightTeleopEnabled.get());
      }

      if (!heightTeleopEnabled.get())
      {
         return;
      }

      ChannelData heightChannel = channelDataMap.get(heightMapping);
      boolean hasNewData = heightChannel.hasNewData();
      double value = heightChannel.getValue();
      if (hasNewData && (value == 0.25 || value == 0.75))
      {
         double heightAdjustment = value == 0.25 ? bodyHeightDeltaPerClick : -bodyHeightDeltaPerClick;
         bodyHeightOffset.add(heightAdjustment);
         bodyHeightOffset.setValue(MathTools.clamp(bodyHeightOffset.getValue(), maximumBodyHeightOffset));
         sendBodyHeightSetpoint();
      }
   }

   private void sendBodyHeightSetpoint()
   {
      ReferenceFrame bodyHeightFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      FramePoint3D bodyHeight = new FramePoint3D(bodyHeightFrame, 0.0, 0.0, nominalBodyHeight + bodyHeightOffset.getValue());
      bodyHeight.changeFrame(ReferenceFrame.getWorldFrame());
      messager.submitMessage(QuadrupedUIMessagerAPI.DesiredBodyHeightTopic, bodyHeight.getZ());
   }

   private void processBodyOrientationCommands()
   {
      ChannelData enableOrientationChannel = channelDataMap.get(enableOrientationTeleop);
      if (enableOrientationChannel.hasNewData() && enableOrientationChannel.getValue() > 0.5)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, !bodyPoseTeleopEnabled.get());
      }

      if (!bodyPoseTeleopEnabled.get())
      {
         return;
      }

      ChannelData negativeYawChannel = channelDataMap.get(negativeYawMapping);
      ChannelData positiveYawChannel = channelDataMap.get(positiveYawMapping);
      ChannelData rollChannel = channelDataMap.get(rollMapping);
      ChannelData pitchChannel = channelDataMap.get(pitchMapping);

      if (negativeYawChannel.hasNewData() || positiveYawChannel.hasNewData() || rollChannel.hasNewData() || pitchChannel.hasNewData())
      {
         double bodyYaw = 0.5 * (positiveYawChannel.getValue() - negativeYawChannel.getValue()) * maxBodyYaw;
         double bodyRoll = rollChannel.getValue() * maxBodyRoll;
         double bodyPitch = pitchChannel.getValue() * maxBodyPitch;
         sendOrientationSetpoint(bodyYaw, bodyRoll, bodyPitch);
      }
   }

   private void sendOrientationSetpoint(double bodyYaw, double bodyRoll, double bodyPitch)
   {
      QuadrupedBodyTrajectoryMessage bodyTrajectoryMessage = new QuadrupedBodyTrajectoryMessage();
      bodyTrajectoryMessage.setIsExpressedInAbsoluteTime(false);
      SE3TrajectoryMessage se3Trajectory = bodyTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      se3Trajectory.getAngularSelectionMatrix().setXSelected(true);
      se3Trajectory.getAngularSelectionMatrix().setYSelected(true);
      se3Trajectory.getAngularSelectionMatrix().setZSelected(true);
      se3Trajectory.getLinearSelectionMatrix().setXSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setYSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setZSelected(false);
      SE3TrajectoryPointMessage trajectoryPointMessage = se3Trajectory.getTaskspaceTrajectoryPoints().add();
      trajectoryPointMessage.getOrientation().setYawPitchRoll(bodyYaw, bodyPitch, bodyRoll);
      trajectoryPointMessage.setTime(bodyOrientationShiftTime);

      messager.submitMessage(QuadrupedUIMessagerAPI.DesiredBodyTrajectoryTopic, bodyTrajectoryMessage);
   }

   private void processXGaitSettingCommands()
   {
      ChannelData increaseEndPhaseShiftChannel = channelDataMap.get(endPhaseShiftUp);
      ChannelData decreaseEndPhaseShiftChannel = channelDataMap.get(endPhaseShiftDown);

      if (increaseEndPhaseShiftChannel.hasNewData() && increaseEndPhaseShiftChannel.getValue() > 0.5)
      {
         xGaitSettings.setEndPhaseShift(xGaitSettings.getEndPhaseShift() + endPhaseDeltaPerClick);
         messager.submitMessage(QuadrupedUIMessagerAPI.XGaitSettingsTopic, xGaitSettings);
      }
      else if (decreaseEndPhaseShiftChannel.hasNewData() && decreaseEndPhaseShiftChannel.getValue() > 0.5)
      {
         xGaitSettings.setEndPhaseShift(xGaitSettings.getEndPhaseShift() - endPhaseDeltaPerClick);
         messager.submitMessage(QuadrupedUIMessagerAPI.XGaitSettingsTopic, xGaitSettings);
      }
   }

   private void processStepCommands()
   {
      ChannelData enableSteppingChannel = channelDataMap.get(enableWalkingTeleop);
      if (enableSteppingChannel.hasNewData() && enableSteppingChannel.getValue() > 0.5)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, !stepTeleopEnabled.get());
      }

      if (!stepTeleopEnabled.get())
      {
         return;
      }

      ChannelData xVelocityChannel = channelDataMap.get(xVelocityMapping);
      ChannelData yVelocityChannel = channelDataMap.get(yVelocityMapping);
      ChannelData leftTurnChannel = channelDataMap.get(leftTurnMapping);
      ChannelData rightTurnChannel = channelDataMap.get(rightTurnMapping);

      if (xVelocityChannel.hasNewData() || yVelocityChannel.hasNewData() || leftTurnChannel.hasNewData() || rightTurnChannel.hasNewData())
      {
         maxVelocityY.setValue(xGaitSettings.getMaxHorizontalSpeedFraction() * xGaitSettings.getMaxSpeed());
         maxVelocityYaw.setValue(xGaitSettings.getMaxYawSpeedFraction() * xGaitSettings.getMaxSpeed());

         double xVelocity = xVelocityChannel.getValue() * xGaitSettings.getMaxSpeed();
         double yVelocity = yVelocityChannel.getValue() * maxVelocityY.getValue();

         // triggers go (-1.0, 1.0). If this is remapped to not be a trigger, this needs to be updated
         double yawRate = 0.5 * (leftTurnChannel.getValue() - rightTurnChannel.getValue()) * maxVelocityYaw.getValue();

         QuadrupedTeleopDesiredVelocity desiredVelocity = new QuadrupedTeleopDesiredVelocity();
         desiredVelocity.setDesiredXVelocity(xVelocity);
         desiredVelocity.setDesiredYVelocity(yVelocity);
         desiredVelocity.setDesiredYawVelocity(yawRate);
         messager.submitMessage(QuadrupedUIMessagerAPI.DesiredTeleopVelocity, desiredVelocity);
      }
   }

   @Override
   public void processEvent(Event event)
   {
      channelDataMap.get(XBoxOneMapping.getMapping(event)).update((double) event.getValue());
   }

   private class ChannelData
   {
      // flag is true when new data is available
      private boolean updateFlag = false;

      private double value = 0.0;
      private double previousValue = 0.0;

      synchronized void update(double value)
      {
         this.updateFlag = true;
         this.previousValue = this.value;
         this.value = value;
      }

      void initialize(double value)
      {
         this.updateFlag = false;
         this.value = value;
         this.previousValue = value;
      }

      synchronized double getValue()
      {
         updateFlag = false;
         return value;
      }

      synchronized boolean hasNewData()
      {
         return updateFlag && (value != previousValue);
      }
   }
}
