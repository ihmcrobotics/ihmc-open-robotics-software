package us.ihmc.rdx.ui.behavior.editor;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import ihmc_common_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.HandWrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXHandWrenchAction extends RDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RobotSide side;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ImDouble trajectoryTime = new ImDouble(1000.0);
   private final ImDouble force = new ImDouble(20.0);

   public RDXHandWrenchAction(ROS2ControllerHelper ros2ControllerHelper)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      ImGui.inputDouble(labels.get("Trajectory time"), trajectoryTime);
      ImGui.inputDouble(labels.get("Force"), force);
      ImGui.popItemWidth();
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
