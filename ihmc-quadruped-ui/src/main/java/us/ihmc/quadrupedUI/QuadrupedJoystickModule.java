package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.QuadrupedTeleopDesiredPose;
import controller_msgs.msg.dds.QuadrupedTeleopDesiredVelocity;
import javafx.animation.AnimationTimer;
import net.java.games.input.Event;
import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commons.Conversions;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.messager.Messager;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.input.InputValueIntegrator;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

import static us.ihmc.quadrupedUI.QuadrupedXBoxBindings.*;

public class QuadrupedJoystickModule extends AnimationTimer implements JoystickEventListener
{
   private static final int pollRateMillis = 50;

   private final MutableDouble maxBodyYaw = new MutableDouble();
   private final MutableDouble maxBodyRoll = new MutableDouble();
   private final MutableDouble maxBodyPitch = new MutableDouble();
   private final MutableDouble maxBodyHeightVelocity = new MutableDouble();
   private final MutableDouble maxTranslationX = new MutableDouble();
   private final MutableDouble maxTranslationY = new MutableDouble();
   private final MutableDouble maxYSpeedFraction = new MutableDouble();
   private final MutableDouble maxYawSpeedFraction = new MutableDouble();
   private final MutableDouble maxVelocityY = new MutableDouble();
   private final MutableDouble maxVelocityYaw = new MutableDouble();
   private final MutableDouble bodyOrientationShiftTime = new MutableDouble();
   private final InputValueIntegrator bodyHeight = new InputValueIntegrator(Conversions.millisecondsToSeconds(pollRateMillis), 0.0);

   private final Messager messager;
   private final Map<XBoxOneMapping, Double> channels = Collections.synchronizedMap(new EnumMap<>(XBoxOneMapping.class));
   private final QuadrupedXGaitSettingsBasics xGaitSettings;
   private final AtomicBoolean joystickPollFlag = new AtomicBoolean();
   private final double nominalBodyHeight;

   private final AtomicBoolean enabled = new AtomicBoolean(false);
   private final AtomicBoolean resetBodyPose = new AtomicBoolean(false);

   public QuadrupedJoystickModule(Messager messager, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double nominalBodyHeight, Joystick joystick)
   {
      this.messager = messager;
      this.xGaitSettings = new QuadrupedXGaitSettings(defaultXGaitSettings);
      this.nominalBodyHeight = nominalBodyHeight;

      joystick.addJoystickEventListener(this);
      joystick.setPollInterval(pollRateMillis);
      configureJoystickFilters(joystick);

      for (XBoxOneMapping channel : XBoxOneMapping.values)
      {
         channels.put(channel, 0.0);
      }

      maxBodyYaw.setValue(0.15);
      maxBodyRoll.setValue(0.15);
      maxBodyPitch.setValue(0.15);
      maxBodyHeightVelocity.setValue(0.1);
      maxTranslationX.setValue(0.25);
      maxTranslationY.setValue(0.15);
      maxYSpeedFraction.setValue(0.5);
      maxYawSpeedFraction.setValue(0.75);
      bodyOrientationShiftTime.setValue(0.1);
      bodyHeight.reset(nominalBodyHeight);

      messager.registerTopicListener(QuadrupedUIMessagerAPI.XGaitSettingsTopic, xGaitSettings::set);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.CurrentControllerNameTopic, state ->
      {
         if (state != HighLevelControllerName.WALKING)
         {
            enabled.set(false);
         }
      });
   }

   private static void configureJoystickFilters(Joystick device)
   {
      device.setCustomizationFilter(new JoystickCustomizationFilter(xVelocityMapping, xVelocityInvert, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(yVelocityMapping, yVelocityInvert, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(negativeYawRateMapping, negativeYawRateInvert, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(positiveYawRateMapping, positiveYawRateInvert, 0.05, 1, 1.0));

      device.setCustomizationFilter(new JoystickCustomizationFilter(xTranslationMapping, xTranslationInvert, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(yTranslationMapping, yTranslationInvert, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(negativeYawMapping, negativeYawInvert, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(positiveYawMapping, positiveYawInvert, 0.05, 1, 1.0));

      device.setCustomizationFilter(new JoystickCustomizationFilter(rollMapping, rollInvert, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(pitchMapping, pitchInvert, 0.1, 1));
   }

   @Override
   public void handle(long now)
   {
      if (!joystickPollFlag.getAndSet(false))
      {
         return;
      }

      if (resetBodyPose.getAndSet(false))
      {
         sendResetCommands();
      }

      if (!enabled.get())
      {
         return;
      }

      messager.submitMessage(QuadrupedUIMessagerAPI.XGaitSettingsTopic, xGaitSettings);

      processJoystickStepCommands();
      processJoystickBodyCommands();
      processJoystickHeightCommands();
   }

   private void sendResetCommands()
   {
      bodyHeight.reset(nominalBodyHeight);

      QuadrupedTeleopDesiredPose desiredPoseMessage = new QuadrupedTeleopDesiredPose();
      desiredPoseMessage.getPose().getPosition().set(0.0, 0.0, bodyHeight.value());
      desiredPoseMessage.getPose().getOrientation().setYawPitchRoll(0.0, 0.0, 0.0);
      desiredPoseMessage.setPoseShiftTime(bodyOrientationShiftTime.getValue());

      messager.submitMessage(QuadrupedUIMessagerAPI.DesiredTeleopBodyPoseTopic, desiredPoseMessage);
      messager.submitMessage(QuadrupedUIMessagerAPI.DesiredBodyHeightTopic, bodyHeight.value());
   }

   private void processJoystickStepCommands()
   {
      maxVelocityY.setValue(maxYSpeedFraction.getValue() * xGaitSettings.getMaxSpeed());
      maxVelocityYaw.setValue(maxYawSpeedFraction.getValue() * xGaitSettings.getMaxSpeed());

      double xVelocity = channels.get(xVelocityMapping) * xGaitSettings.getMaxSpeed();
      double yVelocity = channels.get(yVelocityMapping) * maxVelocityY.getValue();

      double bodyYawLeft = channels.get(negativeYawRateMapping);
      double bodyYawRight = channels.get(positiveYawRateMapping);
      double yawRate = (bodyYawLeft - bodyYawRight) * maxVelocityYaw.getValue();

      QuadrupedTeleopDesiredVelocity desiredVelocity = new QuadrupedTeleopDesiredVelocity();
      desiredVelocity.setDesiredXVelocity(xVelocity);
      desiredVelocity.setDesiredYVelocity(yVelocity);
      desiredVelocity.setDesiredYawVelocity(yawRate);
      messager.submitMessage(QuadrupedUIMessagerAPI.DesiredTeleopVelocity, desiredVelocity);
   }

   private void processJoystickBodyCommands()
   {
      double bodyRoll = channels.get(rollMapping) * maxBodyRoll.getValue();
      double bodyPitch = channels.get(pitchMapping) * maxBodyPitch.getValue();

      double bodyXTranslation = channels.get(xTranslationMapping) * maxTranslationX.getValue();
      double bodyYTranslation = channels.get(yTranslationMapping) * maxTranslationY.getValue();

      double bodyYawLeft = channels.get(negativeYawMapping);
      double bodyYawRight = channels.get(positiveYawMapping);
      double bodyYaw = (bodyYawLeft - bodyYawRight) * maxBodyYaw.getValue();

      QuadrupedTeleopDesiredPose desiredPoseMessage = new QuadrupedTeleopDesiredPose();
      desiredPoseMessage.getPose().getPosition().set(bodyXTranslation, bodyYTranslation, bodyHeight.value());
      desiredPoseMessage.getPose().getOrientation().setYawPitchRoll(bodyYaw, bodyPitch, bodyRoll);
      desiredPoseMessage.setPoseShiftTime(bodyOrientationShiftTime.getValue());
      messager.submitMessage(QuadrupedUIMessagerAPI.DesiredTeleopBodyPoseTopic, desiredPoseMessage);
   }

   private void processJoystickHeightCommands()
   {
      if (channels.get(XBoxOneMapping.DPAD) == 0.25)
      {
         double bodyHeightVelocity = maxBodyHeightVelocity.getValue();
         bodyHeight.update(bodyHeightVelocity);
      }
      else if (channels.get(XBoxOneMapping.DPAD) == 0.75)
      {
         double bodyHeightVelocity = -maxBodyHeightVelocity.getValue();
         bodyHeight.update(bodyHeightVelocity);
      }

      messager.submitMessage(QuadrupedUIMessagerAPI.DesiredBodyHeightTopic, bodyHeight.value());
   }

   private void processStateChangeRequests(Event event)
   {
      if (event.getValue() < 0.5 || xGaitSettings == null)
         return;

      XBoxOneMapping mapping = XBoxOneMapping.getMapping(event);

      if (mapping == endPhaseShiftDown && channels.get(mapping) < 0.5) // the bumpers were firing twice for one click
      {
         xGaitSettings.setEndPhaseShift(xGaitSettings.getEndPhaseShift() - 90.0);
      }
      else if (mapping == endPhaseShiftUp && channels.get(mapping) < 0.5)
      {
         xGaitSettings.setEndPhaseShift(xGaitSettings.getEndPhaseShift() + 90.0);
      }

      if (mapping == XBoxOneMapping.START)
      {
         enabled.set(!enabled.get());
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, false);
      }

      if (mapping == XBoxOneMapping.B)
      {
         resetBodyPose.set(true);
      }
   }

   @Override
   public void processEvent(Event event)
   {
      joystickPollFlag.set(true);
      processStateChangeRequests(event);
      channels.put(XBoxOneMapping.getMapping(event), (double) event.getValue());
   }
}
