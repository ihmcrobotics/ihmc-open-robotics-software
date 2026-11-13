package us.ihmc.rdx.ui.hands.psyonicAbilityHand;

import ihmc_hands_ros2.AbilityHandCommand;
import ihmc_hands_ros2.AbilityHandState;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2API;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.hands.RDXHandInterface;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;

public class RDXAbilityHand implements RDXHandInterface
{
   public static final float START_POSITION = 30.0f;
   public static final float DEFAULT_VELOCITY = 180.0f;
   public static final float THUMB_CURL_MAX = 70.0f;
   public static final float THUMB_CURL_MIN = 10.0f;
   public static final float THUMB_OPPOSITION_MAX = 100.0f;
   public static final float THUMB_OPPOSITION_MIN = 10.0f;
   public static final float FINGER_CURL_MAX = 80.0f;
   public static final float FINGER_CURL_MIN = 0.0f;
   public static final String[] FINGER_NAMES = {"Index", "Middle", "Ring", "Pinky", "Flex", "Rotator"};

   private final RobotSide handSide;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat[] desiredPositions = new ImFloat[6];
   private final ImFloat[] desiredVelocities = new ImFloat[6];
   private AbilityHandGrip executeGrip = null;
   private boolean executePosition = false;

   private final TypedNotification<AbilityHandState> stateNotification = new TypedNotification<>();
   private AbilityHandState latestState = null;
   private final AbilityHandCommand command = new AbilityHandCommand();
   private final ROS2Publisher<AbilityHandCommand> commandPublisher;
   private final Throttler commandThrottler = new Throttler().setFrequency(30.0);
   private final Timer connectedTimer = new Timer();

   public RDXAbilityHand(RobotSide handSide, ROS2Node ros2Node)
   {
      this.handSide = handSide;

      for (int i = 0; i < 6; i++)
      {
         desiredPositions[i] = new ImFloat(AbilityHandGrip.KEY_CLOSE.getStageFingerPosition(0, i));
         desiredVelocities[i] = new ImFloat(DEFAULT_VELOCITY);
      }

      ros2Node.createSubscriptionSampler(AbilityHandROS2API.STATE_TOPICS.get(handSide), sample ->
      {
         AbilityHandState state = new AbilityHandState();
         state.set(sample);
         stateNotification.set(state);
      });
      commandPublisher = ros2Node.createPublisher(AbilityHandROS2API.COMMAND_TOPICS.get(handSide));
   }

   @Override
   public void update()
   {
      if (stateNotification.poll())
      {
         latestState = stateNotification.read();
         connectedTimer.reset();
      }

      if (!connectedTimer.isRunning(0.5))
         latestState = null;

      if (latestState == null)
      {
         executeGrip = null;
         executePosition = false;
      }
      else if ((executeGrip != null || executePosition) && commandThrottler.run())
      {
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
         commandPublisher.publish(command);

         executeGrip = null;
         executePosition = false;
      }
   }

   /** Grips in rows by use, with the row label and each button's text. */
   private static final String[] GRIP_ROW_LABELS = {"Hand", "Grasp", "Lever", "Key"};
   private static final AbilityHandGrip[][] GRIP_ROWS = {{AbilityHandGrip.OPEN, AbilityHandGrip.CLOSE, AbilityHandGrip.RELAX, AbilityHandGrip.FLAT},
                                                         {AbilityHandGrip.PINCH, AbilityHandGrip.HOOK, AbilityHandGrip.HANDLE},
                                                         {AbilityHandGrip.DOOR_LEVER_OPEN, AbilityHandGrip.DOOR_LEVER_CLOSE, AbilityHandGrip.DOOR_LEVER_CRUSH},
                                                         {AbilityHandGrip.KEY_OPEN, AbilityHandGrip.KEY_CLOSE}};
   private static final String[][] GRIP_BUTTON_TEXT = {{"Open", "Close", "Relax", "Flat"},
                                                       {"Pinch", "Hook", "Handle"},
                                                       {"Open", "Close", "Crush"},
                                                       {"Open", "Close"}};
   private static final int GRIP_COLUMNS = 4;

   @Override
   public void renderImGuiWidgets()
   {
      ImGuiTools.separatorText(handSide.getPascalCaseName() + " Ability Hand", ImGuiTools.getSmallBoldFont());
      if (latestState == null)
      {
         ImGui.sameLine();
         ImGui.textDisabled("(not connected)");
      }

      ImGui.beginDisabled(latestState == null);

      float rowStartX = ImGui.getCursorPosX();
      float labelColumnWidth = ImGui.calcTextSize("Rotator").x + ImGui.getStyle().getItemSpacingX() * 2.0f;
      float spacing = ImGui.getStyle().getItemSpacingX();

      // Grips: one row per use, buttons the same width so the columns line up
      float buttonWidth = (ImGui.getContentRegionAvailX() - labelColumnWidth - spacing * (GRIP_COLUMNS - 1)) / GRIP_COLUMNS;
      for (int row = 0; row < GRIP_ROWS.length; row++)
      {
         ImGui.alignTextToFramePadding();
         ImGui.textDisabled(GRIP_ROW_LABELS[row]);
         for (int column = 0; column < GRIP_ROWS[row].length; column++)
         {
            ImGui.sameLine(rowStartX + labelColumnWidth + column * (buttonWidth + spacing));
            if (ImGui.button(labels.get(GRIP_BUTTON_TEXT[row][column], GRIP_ROWS[row][column].name()), buttonWidth, 0.0f))
               executeGrip = GRIP_ROWS[row][column];
         }
      }

      ImGui.spacing();

      // One speed for every finger
      boolean scheduleExecutePosition = false;
      ImGui.alignTextToFramePadding();
      ImGui.textDisabled("Speed");
      ImGui.sameLine(rowStartX + labelColumnWidth);
      ImGui.pushItemWidth(ImGui.getContentRegionAvailX());
      if (ImGuiTools.volatileInputFloat(labels.getHidden("Speed"), desiredVelocities[0], 1.0f, 10.0f, "%.0f deg/s"))
      {
         for (int i = 1; i < 6; i++)
            desiredVelocities[i].set(desiredVelocities[0].get());
         scheduleExecutePosition = true;
      }
      ImGui.popItemWidth();

      // Fingers: the handle is the goal, the notch and the number are where the finger is
      for (int i = 0; i < 6; i++)
      {
         float sliderMin = 0.0f;
         float sliderMax = i == 5 ? -120.0f : 120.0f; // thumb rotator moves negative
         float actuatorPosition = latestState == null ? Float.NaN : latestState.getActuatorPositions()[i];

         ImGui.alignTextToFramePadding();
         ImGui.textDisabled(FINGER_NAMES[i]);
         ImGui.sameLine(rowStartX + labelColumnWidth);

         float sliderWidth = ImGui.getContentRegionAvailX();
         if (!Float.isNaN(actuatorPosition))
         {
            float currentNotch = (actuatorPosition - sliderMin) / (sliderMax - sliderMin);
            ImGuiTools.renderSliderOrProgressNotch(currentNotch * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));
         }
         ImGui.pushItemWidth(sliderWidth);
         String positionText = Float.isNaN(actuatorPosition) ? "--" : "%.0f%s".formatted(actuatorPosition, EuclidCoreMissingTools.DEGREE_SYMBOL);
         scheduleExecutePosition |= ImGui.sliderFloat(labels.getHidden(FINGER_NAMES[i]), desiredPositions[i].getData(), sliderMin, sliderMax, positionText);
         if (!ImGui.isItemActive() && !executePosition && latestState != null) // Prevent overriding externally submitted positions too
            desiredPositions[i].set(latestState.getGoalPositions()[i]);
         ImGui.popItemWidth();
      }

      if (scheduleExecutePosition)
         executePosition = true;
      ImGui.endDisabled();
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
      executePosition = true;
   }

   @Override
   public float getFingerPosition(int index)
   {
      return latestState.getActuatorPositions()[index];
   }
}
