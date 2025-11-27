package us.ihmc.rdx.ui.hands.psyonicAbilityHand;

import com.badlogic.gdx.graphics.Color;
import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
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
   private AbilityHandGrip executeGrip = null;
   private boolean executeVelToPos = false;
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
      if (!communication.getAvailableHands().contains(identifier))
      {
         executeGrip = null; // Clear so they don't get executed later
         executeVelToPos = false;
         return;
      }

      currentPositions = communication.readState(identifier).getActuatorPositions();

      if ((executeGrip != null || executeVelToPos) && publishThrottler.run())
      {
         AbilityHandCommand command = communication.getCommand(identifier);
         if (executeGrip != null)
         {
            command.setControlMode(AbilityHandControlMode.GRIP.toByte());
            command.setGrip(executeGrip.toByte());
         }
         else
         {
            command.setControlMode(AbilityHandControlMode.POSITION.toByte());
            for (int i = 0; i < 6; i++)
               command.getGoalPositions()[i] = desiredPositions[i].get();
         }
         for (int i = 0; i < 6; i++)
            command.getGoalVelocities()[i] = desiredVelocities[i].get();
         communication.publishCommand(identifier);

         executeGrip = null;
         executeVelToPos = false;
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

      for (AbilityHandGrip grip : AbilityHandGrip.values)
      {
         if (ImGui.button(labels.get(grip.name())))
            executeGrip = grip;
         if (grip != AbilityHandGrip.values[5] && grip != AbilityHandGrip.values[7] && grip != AbilityHandGrip.values[AbilityHandGrip.values.length - 1])
            ImGui.sameLine();
      }

      boolean scheduleExecuteVelToPos = false;
      for (int i = 0; i < 6; i++)
      {
         float sliderMin = 0.0f;
         float sliderMax = i == 5 ? -120.0f : 120.0f; // thumb rotator moves negative
         float currentNotch = (currentPositions[i] - sliderMin) / (sliderMax - sliderMin);
         float sliderWidth = ImGui.getColumnWidth() * 0.6f;
         ImGuiTools.renderSliderOrProgressNotch(currentNotch * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

         ImGui.pushItemWidth(sliderWidth);
         scheduleExecuteVelToPos |= ImGui.sliderFloat(labels.getHidden(FINGER_NAMES[i]), desiredPositions[i].getData(), sliderMin, sliderMax,
                               "%s: %.2f%s flexion".formatted(FINGER_NAMES[i], currentPositions[i], EuclidCoreMissingTools.DEGREE_SYMBOL));
         if (!ImGui.isItemActive() && !executeVelToPos) // Prevent overriding externally submitted positions too
            desiredPositions[i].set(currentPositions[i]);
         ImGui.popItemWidth();
         ImGui.sameLine();
         ImGui.pushItemWidth(ImGui.getColumnWidth());
         scheduleExecuteVelToPos |= ImGui.inputFloat(labels.getHidden("Velocity" + i), desiredVelocities[i], 0.1f, 1.0f, "%.2f deg/s");
         ImGui.popItemWidth();
      }

      if (scheduleExecuteVelToPos)
         executeVelToPos = true;
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
         executeGrip = AbilityHandGrip.OPEN;
      else if (handAction == HandAction.CLOSE || handAction == HandAction.GRIP)
         executeGrip = AbilityHandGrip.CLOSE;
      else
         LogTools.warn("Attempted to send an unsupported hand action command: {}", handAction.name());
   }

   @Override
   public void sendFingerPosition(int index, float angleDegrees)
   {
      desiredPositions[index].set(index == 5 ? -angleDegrees : angleDegrees);
      executeVelToPos = true;
   }

   @Override
   public float getFingerPosition(int index)
   {
      return currentPositions[index];
   }
}
