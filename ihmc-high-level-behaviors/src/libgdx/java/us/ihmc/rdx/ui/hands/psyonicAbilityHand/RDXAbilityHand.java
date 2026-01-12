package us.ihmc.rdx.ui.hands.psyonicAbilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2API;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.hands.RDXHandInterface;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
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
         desiredPositions[i] = new ImFloat(START_POSITION);
         desiredVelocities[i] = new ImFloat(DEFAULT_VELOCITY);
      }

      ros2Node.createSubscription2(AbilityHandROS2API.STATE_TOPICS.get(handSide), stateNotification::set);
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

   @Override
   public void renderImGuiWidgets()
   {
      ImGuiTools.separatorText(getSide().toString() + " Ability Hand", ImGuiTools.getSmallBoldFont());

      ImGui.beginDisabled(latestState == null);

      for (AbilityHandGrip grip : AbilityHandGrip.values)
      {
         if (ImGui.button(labels.get(grip.name())))
            executeGrip = grip;
         if (grip != AbilityHandGrip.values[5] && grip != AbilityHandGrip.values[7] && grip != AbilityHandGrip.values[AbilityHandGrip.values.length - 1])
            ImGui.sameLine();
      }

      boolean scheduleExecutePosition = false;
      for (int i = 0; i < 6; i++)
      {
         float sliderMin = 0.0f;
         float sliderMax = i == 5 ? -120.0f : 120.0f; // thumb rotator moves negative
         float actuatorPosition = latestState == null ? Float.NaN : latestState.getActuatorPositions()[i];
         float currentNotch = (actuatorPosition - sliderMin) / (sliderMax - sliderMin);
         float sliderWidth = ImGui.getColumnWidth() * 0.6f;
         ImGuiTools.renderSliderOrProgressNotch(currentNotch * sliderWidth, ImGui.getColorU32(ImGuiCol.Text));

         ImGui.pushItemWidth(sliderWidth);
         scheduleExecutePosition |= ImGui.sliderFloat(labels.getHidden(FINGER_NAMES[i]), desiredPositions[i].getData(), sliderMin, sliderMax,
                               "%s: %.2f%s flexion".formatted(FINGER_NAMES[i], actuatorPosition, EuclidCoreMissingTools.DEGREE_SYMBOL));
         if (!ImGui.isItemActive() && !executePosition && latestState != null) // Prevent overriding externally submitted positions too
            desiredPositions[i].set(latestState.getGoalPositions()[i]);
         ImGui.popItemWidth();
         ImGui.sameLine();
         ImGui.pushItemWidth(ImGui.getColumnWidth());
         boolean velocityInput = ImGuiTools.volatileInputFloat(labels.getHidden("Velocity" + i), desiredVelocities[i], 0.1f, 1.0f, "%.2f deg/s");
         if (velocityInput)
            for (int j = 0; j < 6; j++)
               desiredVelocities[j].set(desiredVelocities[i].get());
         scheduleExecutePosition |= velocityInput;
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
