package us.ihmc.rdx.ui.hands.psyonicAbilityHand;

import com.badlogic.gdx.graphics.Color;
import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.Grip;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2HardwareCommunication;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.hands.RDXHandInterface;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXAbilityHand implements RDXHandInterface
{
   public static final float START_POSITION = 30.0f;
   public static final float SLIDER_MIN = 0.0f;
   public static final float SLIDER_MAX = 120.0f;
   public static final float DEFAULT_VELOCITY = 30.0f; // TODO: Is 30 a good default?
   public static final float THUMB_CURL_MAX = 70.0f;
   public static final float THUMB_CURL_MIN = 10.0f;
   public static final float THUMB_OPPOSITION_MAX = 100.0f;
   public static final float THUMB_OPPOSITION_MIN = 10.0f;
   public static final float FINGER_CURL_MAX = 80.0f;
   public static final float FINGER_CURL_MIN = 0.0f;
   public static final String[] FINGER_NAMES = {"Index", "Middle", "Ring", "Pinky", "Flex", "Rotator"};

   private final String identifier;
   private final RobotSide handSide;
   private final AbilityHandROS2HardwareCommunication communication;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat[] desiredPositions = new ImFloat[6];
   private final ImFloat[] desiredVelocities = new ImFloat[6];

   private float[] currentPositions = new float[6];
   private final TypedNotification<Grip> gripNotification = new TypedNotification<>();
   private final Notification velToPosNotification = new Notification();
   private final Throttler publishThrottler = new Throttler().setFrequency(30.0);

   public RDXAbilityHand(String identifier, RobotSide handSide, AbilityHandROS2HardwareCommunication communication)
   {
      this.identifier = identifier;
      this.handSide = handSide;
      this.communication = communication;

      for (int i = 0; i < 6; i++)
      {
         desiredPositions[i] = new ImFloat(START_POSITION);
         desiredVelocities[i] = new ImFloat(DEFAULT_VELOCITY);
      }
   }

   @Override
   public void update()
   {
      boolean executeGrip = gripNotification.poll();
      boolean executeVelToPos = velToPosNotification.poll();

      if (!communication.getAvailableHands().contains(identifier))
         return;

      currentPositions = communication.readState(identifier).getActuatorPositions();

      if ((executeGrip || executeVelToPos) && publishThrottler.run())
      {
         AbilityHandCommand command = communication.getCommand(identifier);
         if (executeGrip)
         {
            command.setControlMode(ControlMode.GRIP.toByte());
            command.setGrip(gripNotification.read().toByte());
         }
         else
         {
            command.setControlMode(ControlMode.VEL_TO_POS.toByte());
            for (int i = 0; i < 6; i++)
               command.getGoalPositions()[i] = desiredPositions[i].get();
         }
         for (int i = 0; i < 6; i++)
            command.getGoalVelocities()[i] = desiredVelocities[i].get();
         communication.publishCommand(identifier);
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      boolean connected = communication.getAvailableHands().contains(identifier);

      ImGuiTools.separatorText(getSide().toString() + " Ability Hand", ImGuiTools.getSmallBoldFont());

      if (!connected)
         ImGuiTools.textColored(Color.RED, "Not connected");

      ImGui.beginDisabled(!connected);

      if (ImGui.button("Open"))
         gripNotification.set(Grip.RELAX);
      ImGui.sameLine();
      if (ImGui.button("Grip"))
         gripNotification.set(Grip.POWER);
      ImGui.sameLine();
      if (ImGui.button("Tripod Closed"))
         gripNotification.set(Grip.TRIPOD_C);
      ImGui.sameLine();
      if (ImGui.button("Hook"))
         gripNotification.set(Grip.HOOK);
      ImGui.sameLine();
      if (ImGui.button("Tripod Open"))
         gripNotification.set(Grip.TRIPOD_O);
      if (ImGui.button("Pinch Open"))
         gripNotification.set(Grip.PINCH_O);
      ImGui.sameLine();
      if (ImGui.button("Pinch Closed"))
         gripNotification.set(Grip.PINCH_C);
      ImGui.sameLine();
      if (ImGui.button("Key"))
         gripNotification.set(Grip.KEY);
      ImGui.sameLine();
      if (ImGui.button("Rude"))
         gripNotification.set(Grip.RUDE);

      boolean executeVelToPos = false;
      for (int i = 0; i < 6; i++)
      {
         float currentNotch = (currentPositions[0] - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN);
         float sliderWidth = ImGui.getColumnWidth();
         ImGuiTools.renderSliderOrProgressNotch(currentNotch * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

         ImGui.pushItemWidth(ImGui.getColumnWidth() * 0.6f);
         executeVelToPos |= ImGui.sliderFloat(labels.getHidden(FINGER_NAMES[i]), desiredPositions[i].getData(), SLIDER_MIN, SLIDER_MAX,
                               "%s: %.2f%s flexion".formatted(FINGER_NAMES[i], desiredPositions[i].get(), EuclidCoreMissingTools.DEGREE_SYMBOL));
         ImGui.popItemWidth();
         ImGui.sameLine();
         ImGui.pushItemWidth(ImGui.getColumnWidth());
         executeVelToPos |= ImGui.inputFloat(labels.getHidden("Velocity" + i), desiredVelocities[i], 0.1f, 1.0f, "%.2f rad/s");
         ImGui.popItemWidth();
      }

      if (executeVelToPos)
         velToPosNotification.set();
      ImGui.endDisabled();
   }

   @Override
   public String getIdentifier()
   {
      return identifier;
   }

   @Override
   public RobotSide getSide()
   {
      return handSide;
   }

   @Override
   public boolean isCalibrated()
   {
      return true;
   }

   @Override
   public boolean needsReset()
   {
      return false;
   }

   @Override
   public void sendCommand(HandAction handAction)
   {
      if (handAction == HandAction.OPEN)
         gripNotification.set(Grip.RELAX);
      else if (handAction == HandAction.CLOSE || handAction == HandAction.GRIP)
         gripNotification.set(Grip.POWER);
      else
         LogTools.warn("Attempted to send an unsupported hand action command: {}", handAction.name());
   }

   @Override
   public void sendFingerPosition(int index, float value)
   {
      desiredPositions[index].set(value);
      velToPosNotification.set();
   }

   @Override
   public float getFingerPosition(int index)
   {
      return currentPositions[index];
   }
}
