package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.tools.ImGuiRobotSideCombo;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXArmJointAnglesAction implements GDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean selected = new ImBoolean();
   private ImGuiRobotSideCombo side = new ImGuiRobotSideCombo();
   private ROS2ControllerHelper ros2ControllerHelper;
   private int numberOfJoints = 7;
   private ImDouble[] jointAngles = new ImDouble[numberOfJoints];
   private final ImDouble trajectoryTime = new ImDouble(4.0);

   public GDXArmJointAnglesAction()
   {
      for (int i = 0; i < numberOfJoints; i++)
      {
         jointAngles[i] = new ImDouble();
      }
   }

   public void create(ROS2ControllerHelper ros2ControllerHelper)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public void update()
   {

   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {

   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {

   }

   @Override
   public void renderImGuiWidgets()
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
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

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

   @Override
   public void destroy()
   {

   }

   @Override
   public ImBoolean getSelected()
   {
      return selected;
   }

   @Override
   public String getNameForDisplay()
   {
      return "Arm Joint Angles";
   }
}
