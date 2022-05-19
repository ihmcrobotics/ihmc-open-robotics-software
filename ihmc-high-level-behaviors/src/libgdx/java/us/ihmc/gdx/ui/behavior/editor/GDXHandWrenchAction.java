package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.HandWrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXHandWrenchAction implements GDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RobotSide side;
   private ROS2ControllerHelper ros2ControllerHelper;
   private final ImBoolean selected = new ImBoolean();
   private final ImDouble trajectoryTime = new ImDouble(1000.0);
   private final ImDouble force = new ImDouble(20.0);

   public void create(ROS2ControllerHelper ros2ControllerHelper)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
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
      ImGui.pushItemWidth(80.0f);
      ImGui.inputDouble(labels.get("Trajectory time"), trajectoryTime);
      ImGui.inputDouble(labels.get("Force"), force);
      ImGui.popItemWidth();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryTime", trajectoryTime.get());
      jsonNode.put("force", force.get());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      setSide(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      trajectoryTime.set(jsonNode.get("trajectoryTime").asDouble());
      force.set(jsonNode.get("force").asDouble());
   }

   @Override
   public void destroy()
   {

   }

   @Override
   public void performAction()
   {
      HandWrenchTrajectoryMessage handWrenchTrajectoryMessage = new HandWrenchTrajectoryMessage();
      handWrenchTrajectoryMessage.setRobotSide(side.toByte());
//      double force = 4.2; // For 0.5 kg box
      double force = this.force.get();
      if (force > 0.0)
      {
         IDLSequence.Object<WrenchTrajectoryPointMessage> wrenchTrajectoryPoints
               = handWrenchTrajectoryMessage.getWrenchTrajectory().getWrenchTrajectoryPoints();

         double time0 = 0.0;
         Vector3D torque0 = new Vector3D();
         Vector3D force0 = new Vector3D(0.0, side == RobotSide.RIGHT ? force : -force, 0.0);
         wrenchTrajectoryPoints.add().set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(time0, torque0, force0));

         double time1 = trajectoryTime.get();
         Vector3D torque1 = new Vector3D();
         Vector3D force1 = new Vector3D(0.0, side == RobotSide.RIGHT ? force : -force, 0.0);
         wrenchTrajectoryPoints.add().set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(time1, torque1, force1));
      }
      handWrenchTrajectoryMessage.getWrenchTrajectory().getFrameInformation().setTrajectoryReferenceFrameId(FrameInformation.CHEST_FRAME);
      handWrenchTrajectoryMessage.getWrenchTrajectory().setUseCustomControlFrame(true);
      double handCenterOffset = 0.05;
      handWrenchTrajectoryMessage.getWrenchTrajectory().getControlFramePose().setY(side == RobotSide.RIGHT ? -handCenterOffset : handCenterOffset);

      ros2ControllerHelper.publishToController(handWrenchTrajectoryMessage);
   }

   @Override
   public ImBoolean getSelected()
   {
      return selected;
   }

   public RobotSide getSide()
   {
      return side;
   }

   @Override
   public String getNameForDisplay()
   {
      return side.getPascalCaseName() + " Hand Wrench";
   }
}
