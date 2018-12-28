package us.ihmc.quadrupedPlanning.networkProcessing.xBox;

import controller_msgs.msg.dds.*;
import net.java.games.input.Event;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.input.InputValueIntegrator;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedRobotModelProviderNode;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedPlanning.networkProcessing.bodyTeleop.QuadrupedBodyTeleopModule;
import us.ihmc.quadrupedPlanning.networkProcessing.heightTeleop.QuadrupedBodyHeightTeleopModule;
import us.ihmc.quadrupedPlanning.networkProcessing.stepTeleop.QuadrupedStepTeleopManager;
import us.ihmc.quadrupedPlanning.networkProcessing.stepTeleop.QuadrupedStepTeleopModule;
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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedXBoxController extends QuadrupedToolboxController implements JoystickEventListener
{
   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   private final AtomicBoolean receivedInput = new AtomicBoolean();

   private final Map<XBoxOneMapping, Double> channels = Collections.synchronizedMap(new EnumMap<>(XBoxOneMapping.class));

   private final YoBoolean isPaused = new YoBoolean("xBoxIsPaused", registry);
   private final YoDouble maxBodyYaw = new YoDouble("xBoxMaxBodyYaw", registry);
   private final YoDouble maxBodyPitch = new YoDouble("xBoxMaxBodyPitch", registry);
   private final YoDouble maxBodyHeightVelocity = new YoDouble("xBoxMaxBodyHeightVelocity", registry);
   private final YoDouble maxVelocityX = new YoDouble("xBoxMaxVelocityX", registry);
   private final YoDouble maxVelocityY = new YoDouble("xBoxMaxVelocityY", registry);
   private final YoDouble maxVelocityYaw = new YoDouble("xBoxMaxVelocityYaw", registry);
   private final YoDouble bodyOrientationShiftTime = new YoDouble("xBoxBodyOrientationShiftTime", registry);

   private final InputValueIntegrator bodyHeight;
   private double endPhaseShift;

   private QuadrupedTeleopDesiredVelocity desiredVelocityMessage = null;
   private QuadrupedTeleopDesiredPose desiredPoseMessage = null;
   private QuadrupedTeleopDesiredHeight desiredHeightMessage = null;

   public QuadrupedXBoxController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double nominalBodyHeight, CommandInputManager commandInputManager,
                                  StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry, int updateTimeInMs) throws IOException
   {
      super(statusOutputManager, parentRegistry);

      boolean joystickIsConnected = Joystick.isAJoystickConnectedToSystem();
      if (!joystickIsConnected)
      {
         throw new RuntimeException("No joystick detected, can't run without it.");
      }
      Joystick joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
      configureJoystickFilters(joystick);
      joystick.addJoystickEventListener(this);
      joystick.setPollInterval(10);

      maxBodyYaw.set(0.15);
      maxBodyPitch.set(0.15);
      maxBodyHeightVelocity.set(0.1);
      maxVelocityX.set(0.5);
      maxVelocityY.set(0.25);
      maxVelocityYaw.set(0.4);
      bodyOrientationShiftTime.set(0.1);
      this.bodyHeight = new InputValueIntegrator(Conversions.millisecondsToSeconds(updateTimeInMs), nominalBodyHeight);
      this.endPhaseShift = defaultXGaitSettings.getEndPhaseShift();

      // Initialize all channels to zero.
      for (XBoxOneMapping channel : XBoxOneMapping.values)
      {
         channels.put(channel, 0.0);
      }

      commandInputManager.registerHasReceivedInputListener(command -> receivedInput.set(true));
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void setPaused(boolean paused)
   {
      isPaused.set(paused);
   }

   @Override
   public boolean initialize()
   {
      return true;
   }

   @Override
   public void updateInternal()
   {
      desiredVelocityMessage = null;
      desiredPoseMessage = null;
      desiredHeightMessage = null;

      processJoystickHeightCommands();
      processJoystickBodyCommands();
      processJoystickStepCommands();

      reportMessage(desiredVelocityMessage);
      reportMessage(desiredPoseMessage);
      reportMessage(desiredHeightMessage);
   }

   @Override
   public boolean isDone()
   {
      return controllerStateChangeMessage.get().getEndHighLevelControllerName() != HighLevelStateChangeStatusMessage.WALKING;
   }

   private void configureJoystickFilters(Joystick device)
   {
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_TRIGGER, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_TRIGGER, false, 0.05, 1, 1.0));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_X, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.LEFT_STICK_Y, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_X, true, 0.1, 1));
      device.setCustomizationFilter(new JoystickCustomizationFilter(XBoxOneMapping.RIGHT_STICK_Y, true, 0.1, 1));
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
      double bodyRoll = 0.0;
      double bodyPitch = channels.get(XBoxOneMapping.RIGHT_STICK_Y) * maxBodyPitch.getValue();
      double bodyYaw = channels.get(XBoxOneMapping.RIGHT_STICK_X) * maxBodyYaw.getValue();

      desiredPoseMessage = new QuadrupedTeleopDesiredPose();
      desiredPoseMessage.getPose().getPosition().set(0.0, 0.0, bodyHeight.value());
      desiredPoseMessage.getPose().getOrientation().setYawPitchRoll(bodyYaw, bodyPitch, bodyRoll);
      desiredPoseMessage.setPoseShiftTime(bodyOrientationShiftTime.getValue());
   }

   private void processJoystickStepCommands()
   {
      double xVelocity = channels.get(XBoxOneMapping.LEFT_STICK_Y) * maxVelocityX.getDoubleValue();
      double yVelocity = channels.get(XBoxOneMapping.LEFT_STICK_X) * maxVelocityY.getDoubleValue();
      double yawRate = channels.get(XBoxOneMapping.RIGHT_STICK_X) * maxVelocityYaw.getDoubleValue();

      desiredVelocityMessage = new QuadrupedTeleopDesiredVelocity();
      desiredVelocityMessage.setDesiredXVelocity(xVelocity);
      desiredVelocityMessage.setDesiredYVelocity(yVelocity);
      desiredVelocityMessage.setDesiredYawVelocity(yawRate);
   }

   private void processStateChangeRequests(Event event)
   {
      if (event.getValue() < 0.5)
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
      //      if (mapping == XBoxOneMapping.LEFT_BUMPER && channels.get(mapping) < 0.5 && stepTeleopModule != null) // the bumpers were firing twice for one click
      //      {
      //         endPhaseShift -= 90.0;
      //         stepTeleopModule.setEndPhaseShift(endPhaseShift);
      //      }
      //      else if (mapping == XBoxOneMapping.RIGHT_BUMPER && channels.get(mapping) < 0.5 && stepTeleopModule != null)
      //      {
      //         endPhaseShift += 90.0;
      //         stepTeleopModule.setEndPhaseShift(endPhaseShift);
      //      }
      //      else if(mapping == XBoxOneMapping.XBOX_BUTTON && channels.get(mapping) < 0.5)
      //      {
      //         stepTeleopManager.setPaused(!stepTeleopManager.isPaused());
      //      }
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
