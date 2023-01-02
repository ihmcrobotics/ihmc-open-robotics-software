package us.ihmc.rdx.ui.behavior.editor.actions;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXPelvisHeightAction extends RDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ImDouble heightInWorld = new ImDouble();
   private final ImDouble trajectoryDuration = new ImDouble(4.0);

   public RDXPelvisHeightAction(ROS2ControllerHelper ros2ControllerHelper)
   {
      super("Pelvis Height");

      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      ImGui.inputDouble(labels.get("Height in world"), heightInWorld);
      ImGui.inputDouble(labels.get("Trajectory duration"), trajectoryDuration);
      ImGui.popItemWidth();
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("trajectoryDuration", trajectoryDuration.get());
      jsonNode.put("heightInWorld", heightInWorld.get());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      trajectoryDuration.set(jsonNode.get("trajectoryDuration").asDouble());
      heightInWorld.set(jsonNode.get("heightInWorld").asDouble());
   }

   @Override
   public void performAction()
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory()
             .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryDuration.get(),
                                                                        new Point3D(0.0, 0.0, heightInWorld.get()),
                                                                        ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);

      ros2ControllerHelper.publishToController(message);
   }
}
