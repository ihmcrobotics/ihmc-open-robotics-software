package us.ihmc.rdx.ui.behavior.editor;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.tools.ImGuiRobotSideCombo;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXArmJointAnglesAction extends RDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiRobotSideCombo side = new ImGuiRobotSideCombo();
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final int numberOfJoints = 7;
   private final ImDouble[] jointAngles = new ImDouble[numberOfJoints];
   private final ImDouble trajectoryTime = new ImDouble(4.0);

   public RDXArmJointAnglesAction(ROS2ControllerHelper ros2ControllerHelper)
   {
      super("Arm Joint Angles");

      this.ros2ControllerHelper = ros2ControllerHelper;

      for (int i = 0; i < numberOfJoints; i++)
      {
         jointAngles[i] = new ImDouble();
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(100.0f);
      side.combo(labels.get("Side"));
      ImGui.popItemWidth();
      ImGui.pushItemWidth(80.0f);
      ImGui.inputDouble(labels.get("Trajectory time"), trajectoryTime);
      for (int i = 0; i < numberOfJoints; i++)
      {
         ImGui.inputDouble(labels.get("j" + i), jointAngles[i]);
      }
      ImGui.popItemWidth();
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("side", side.getSide().getLowerCaseName());
      jsonNode.put("trajectoryTime", trajectoryTime.get());
      for (int i = 0; i < numberOfJoints; i++)
      {
         jsonNode.put("j" + i, jointAngles[i].get());
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      side.setSide(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      trajectoryTime.set(jsonNode.get("trajectoryTime").asDouble());
      for (int i = 0; i < numberOfJoints; i++)
      {
         jointAngles[i].set(jsonNode.get("j" + i).asDouble());
      }
   }

   @Override
   public void performAction()
   {
      double[] jointAngleArray = new double[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
      {
         jointAngleArray[i] = jointAngles[i].get();
      }
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side.getSide(), trajectoryTime.get(), jointAngleArray);
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
   }
}
