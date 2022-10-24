package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.tools.ImGuiRobotSideCombo;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXHandConfigurationAction implements GDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiRobotSideCombo side = new ImGuiRobotSideCombo();
   private ROS2ControllerHelper ros2ControllerHelper;
   private final ImInt handConfigurationIndex = new ImInt(6);
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];
   private final ImBoolean selected = new ImBoolean();

   public GDXHandConfigurationAction()
   {
      HandConfiguration[] values = HandConfiguration.values;
      for (int i = 0; i < values.length; i++)
      {
         handConfigurationNames[i] = values[i].name();
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
      ImGui.combo(labels.get("Grip"), handConfigurationIndex, handConfigurationNames);
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
      jsonNode.put("grip", HandConfiguration.values[handConfigurationIndex.get()].name());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      side.setSide(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      handConfigurationIndex.set(HandConfiguration.valueOf(jsonNode.get("grip").asText()).ordinal());
   }

   @Override
   public void performAction()
   {
      HandDesiredConfigurationMessage message
            = HumanoidMessageTools.createHandDesiredConfigurationMessage(side.getSide(),
                                                                         HandConfiguration.values[handConfigurationIndex.get()]);
      ros2ControllerHelper.publish(ROS2Tools::getHandConfigurationTopic, message);
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
      return "Hand Configuration";
   }
}
