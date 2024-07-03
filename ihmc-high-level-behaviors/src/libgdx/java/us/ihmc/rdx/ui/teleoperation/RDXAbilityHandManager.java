package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage;
import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.abilityhand.AbilityHandLegacyGripCommand.LegacyGripSpeed;
import us.ihmc.abilityhand.AbilityHandLegacyGripCommand.LegacyGripType;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.AbilityHandAPI;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXAbilityHandManager
{
   private final ROS2ControllerHelper ros2Helper;
   private final ROS2SyncedRobotModel syncedRobotModel;

   private final SideDependentList<ImInt> currentLegacyGripCommandIndex = new SideDependentList<>();
   private final SideDependentList<ImInt> currentLegacyGripSpeedIndex = new SideDependentList<>();

   private final String[] legacyGripCommandNames;
   private final String[] legacyGripSpeedNames;

   public RDXAbilityHandManager(ROS2ControllerHelper ros2Helper, ROS2SyncedRobotModel syncedRobotModel)
   {
      this.ros2Helper = ros2Helper;
      this.syncedRobotModel = syncedRobotModel;

      legacyGripCommandNames = new String[LegacyGripType.values().length];
      for (int i = 0; i < LegacyGripType.values().length; i++)
         legacyGripCommandNames[i] = LegacyGripType.values()[i].name();

      legacyGripSpeedNames = new String[LegacyGripSpeed.values().length];
      for (int i = 0; i < LegacyGripSpeed.values().length; i++)
         legacyGripSpeedNames[i] = LegacyGripSpeed.values()[i].name();

      currentLegacyGripCommandIndex.put(RobotSide.LEFT, new ImInt());
      currentLegacyGripSpeedIndex.put(RobotSide.LEFT, new ImInt());

      currentLegacyGripCommandIndex.put(RobotSide.RIGHT, new ImInt());
      currentLegacyGripSpeedIndex.put(RobotSide.RIGHT, new ImInt());
   }

   public void renderImGuiWidgets()
   {
      ImGuiTools.separatorText("PSYONIC ability hands");

      ImGui.text("(Legacy) Grip commands");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.text(side.getCamelCaseNameForMiddleOfExpression());

         ImGui.combo("Grip command##AbilityHand" + side.getLowerCaseName(), currentLegacyGripCommandIndex.get(side), legacyGripCommandNames);
         ImGui.combo("Grip speed##AbilityHand" + side.getLowerCaseName(), currentLegacyGripSpeedIndex.get(side), legacyGripSpeedNames);

         if (ImGui.button("Send##AbilityHand" + side.getLowerCaseName()))
         {
            LegacyGripType gripType = LegacyGripType.valueOf(legacyGripCommandNames[currentLegacyGripCommandIndex.get(side).get()]);
            LegacyGripSpeed gripSpeed = LegacyGripSpeed.valueOf(legacyGripSpeedNames[currentLegacyGripSpeedIndex.get(side).get()]);
            sendLegacyGrip(gripType, gripSpeed, side);
         }
      }

      ImGui.separator();
   }

   private void sendLegacyGrip(LegacyGripType gripType, LegacyGripSpeed gripSpeed, RobotSide hand)
   {
      AbilityHandLegacyGripCommandMessage message = new AbilityHandLegacyGripCommandMessage();
      message.setLegacyGripType(gripType.getAsciiGripIndex());
      message.setLegacyGripSpeed(gripSpeed.name());

      ros2Helper.publish(AbilityHandAPI.getAbilityHandLegacyGripCommandTopic(syncedRobotModel.getRobotModel().getSimpleRobotName(), hand), message);
   }
}
