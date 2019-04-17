package us.ihmc.quadrupedCommunication.networkProcessing.xBox;

import controller_msgs.msg.dds.*;
import net.java.games.input.Event;
import us.ihmc.commons.Conversions;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.stepStream.input.InputValueIntegrator;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickCustomizationFilter;
import us.ihmc.tools.inputDevices.joystick.JoystickEventListener;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.mapping.XBoxOneMapping;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;
import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.quadrupedCommunication.networkProcessing.xBox.QuadrupedXBoxBindings.*;

public class QuadrupedXBoxController extends QuadrupedToolboxController implements JoystickEventListener
{
   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   private final Map<XBoxOneMapping, Double> channels = Collections.synchronizedMap(new EnumMap<>(XBoxOneMapping.class));

   private final YoBoolean isPaused = new YoBoolean("xBoxIsPaused", registry);

   private final YoDouble maxBodyYaw = new YoDouble("xBoxMaxBodyYaw", registry);
   private final YoDouble maxBodyRoll = new YoDouble("xBoxMaxBodyRoll", registry);
   private final YoDouble maxBodyPitch = new YoDouble("xBoxMaxBodyPitch", registry);
   private final YoDouble maxBodyHeightVelocity = new YoDouble("xBoxMaxBodyHeightVelocity", registry);

   private final YoDouble maxTranslationX = new YoDouble("xBoxMaxTranslationX", registry);
   private final YoDouble maxTranslationY = new YoDouble("xBoxMaxTranslationY", registry);

   private final YoDouble maxYSpeedFraction = new YoDouble("maxYSpeedFraction", registry);
   private final YoDouble maxYawSpeedFraction = new YoDouble("maxYawSpeedFraction", registry);
   private final YoDouble maxVelocityY = new YoDouble("xBoxMaxVelocityY", registry);
   private final YoDouble maxVelocityYaw = new YoDouble("xBoxMaxVelocityYaw", registry);

   private final YoDouble bodyOrientationShiftTime = new YoDouble("xBoxBodyOrientationShiftTime", registry);

   private final QuadrupedXGaitSettingsBasics xGaitSettings;

   private final InputValueIntegrator bodyHeight;

   private QuadrupedXGaitSettingsPacket xGaitSettingsPacket = null;
   private QuadrupedTeleopDesiredVelocity desiredVelocityMessage = null;
   private QuadrupedTeleopDesiredPose desiredPoseMessage = null;
   private QuadrupedTeleopDesiredHeight desiredHeightMessage = null;

   public QuadrupedXBoxController(QuadrupedRobotDataReceiver robotDataReceiver, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double nominalBodyHeight, OutputManager statusOutputManager,
                                  YoVariableRegistry parentRegistry, int updateTimeInMs) throws IOException
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      boolean joystickIsConnected = Joystick.isAJoystickConnectedToSystem();
      if (!joystickIsConnected)
      {
         throw new RuntimeException("No joystick detected, can't run without it.");
      }
      Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
      configureJoystickFilters(joystick);
      joystick.addJoystickEventListener(this);
      joystick.setPollInterval(10);

      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      maxBodyYaw.set(0.15);
      maxBodyRoll.set(0.15);
      maxBodyPitch.set(0.15);
      maxBodyHeightVelocity.set(0.1);

      maxTranslationX.set(0.25);
      maxTranslationY.set(0.15);

      maxYSpeedFraction.set(0.5);
      maxYawSpeedFraction.set(0.75);

      bodyOrientationShiftTime.set(0.1);
      this.bodyHeight = new InputValueIntegrator(Conversions.millisecondsToSeconds(updateTimeInMs), nominalBodyHeight);

      // Initialize all channels to zero.
      for (XBoxOneMapping channel : XBoxOneMapping.values)
      {
         channels.put(channel, 0.0);
      }
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      xGaitSettings.set(packet);
   }

   public void setPaused(boolean paused)
   {
      isPaused.set(paused);
   }

   @Override
   public boolean initializeInternal()
   {
      return true;
   }

   @Override
   public void updateInternal()
   {
      xGaitSettingsPacket = null;
      desiredVelocityMessage = null;
      desiredPoseMessage = null;
      desiredHeightMessage = null;

      processJoystickHeightCommands();
      processJoystickBodyCommands();
      processJoystickStepCommands();

      reportMessage(xGaitSettingsPacket);
      reportMessage(desiredVelocityMessage);
      reportMessage(desiredPoseMessage);
      reportMessage(desiredHeightMessage);
   }

   @Override
   public boolean isDone()
   {
      if (controllerStateChangeMessage.get() == null)
         return false;

      return controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING;
   }

   private void configureJoystickFilters(Joystick device)
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
      desiredHeightMessage = new QuadrupedTeleopDesiredHeight();
      desiredHeightMessage.setDesiredHeight(bodyHeight.value());
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

      desiredPoseMessage = new QuadrupedTeleopDesiredPose();
      desiredPoseMessage.getPose().getPosition().set(bodyXTranslation, bodyYTranslation, bodyHeight.value());
      desiredPoseMessage.getPose().getOrientation().setYawPitchRoll(bodyYaw, bodyPitch, bodyRoll);
      desiredPoseMessage.setPoseShiftTime(bodyOrientationShiftTime.getValue());
   }

   private void processJoystickStepCommands()
   {
      maxVelocityY.set(maxYSpeedFraction.getDoubleValue() * xGaitSettings.getMaxSpeed());
      maxVelocityYaw.set(maxYawSpeedFraction.getDoubleValue() * xGaitSettings.getMaxSpeed());


      double xVelocity = channels.get(xVelocityMapping) * xGaitSettings.getMaxSpeed();
      double yVelocity = channels.get(yVelocityMapping) * maxVelocityY.getValue();

      double bodyYawLeft = channels.get(negativeYawRateMapping);
      double bodyYawRight = channels.get(positiveYawRateMapping);
      double yawRate = (bodyYawLeft - bodyYawRight) * maxVelocityYaw.getValue();

      desiredVelocityMessage = new QuadrupedTeleopDesiredVelocity();
      desiredVelocityMessage.setDesiredXVelocity(xVelocity);
      desiredVelocityMessage.setDesiredYVelocity(yVelocity);
      desiredVelocityMessage.setDesiredYawVelocity(yawRate);
   }

   private void processStateChangeRequests(Event event)
   {
      if (event.getValue() < 0.5 || xGaitSettings == null)
         return;

      XBoxOneMapping mapping = XBoxOneMapping.getMapping(event);

      //      if (mapping == XBoxOneMapping.START)
      //      {
      //         stepTeleopManager.requestStandPrep();
      //      }
      //      else if (mapping == XBoxOneMapping.A)
      //      {
      //         stepTeleopManager.requestWalkingState();
      //         if (stepTeleopManager.isWalking())
      //            stepTeleopManager.requestStanding();
      //      }
      //      else if (mapping == XBoxOneMapping.X)
      //      {
      //         stepTeleopManager.requestXGait();
      //      }
      if (mapping == endPhaseShiftDown && channels.get(mapping) < 0.5) // the bumpers were firing twice for one click
      {
         xGaitSettings.setEndPhaseShift(xGaitSettings.getEndPhaseShift() - 90.0);
      }
      else if (mapping == endPhaseShiftUp && channels.get(mapping) < 0.5)
      {
         xGaitSettings.setEndPhaseShift(xGaitSettings.getEndPhaseShift() + 90.0);
      }
      //      else if(mapping == XBoxOneMapping.XBOX_BUTTON && channels.get(mapping) < 0.5)
      //      {
      //         stepTeleopManager.setPaused(!stepTeleopManager.isPaused());
      //      }

      xGaitSettingsPacket = xGaitSettings.getAsPacket();
   }

   @Override
   public void processEvent(Event event)
   {
      // Handle events that should trigger once immediately after the event is triggered.
      processStateChangeRequests(event);

      // Store updated value in a cache so historical values for all channels can be used.
      channels.put(XBoxOneMapping.getMapping(event), (double) event.getValue());
   }
}
