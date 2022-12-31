package us.ihmc.rdx.ui.behavior.editor;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

public class RDXChestOrientationAction extends RDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImDouble yaw = new ImDouble();
   private final ImDouble pitch = new ImDouble();
   private final ImDouble roll = new ImDouble();
   private final ImDouble trajectoryTime = new ImDouble(4.0);

   public RDXChestOrientationAction(ROS2ControllerHelper ros2ControllerHelper, ROS2SyncedRobotModel syncedRobot)
   {
      super("Chest Orientation");

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      ImGui.inputDouble(labels.get("Yaw"), yaw);
      ImGui.sameLine();
      ImGui.inputDouble(labels.get("Pitch"), pitch);
      ImGui.sameLine();
      ImGui.inputDouble(labels.get("Roll"), roll);
      ImGui.inputDouble(labels.get("Trajectory time"), trajectoryTime);
      ImGui.popItemWidth();
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("trajectoryTime", trajectoryTime.get());
      jsonNode.put("yaw", yaw.get());
      jsonNode.put("pitch", pitch.get());
      jsonNode.put("roll", roll.get());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      trajectoryTime.set(jsonNode.get("trajectoryTime").asDouble());
      yaw.set(jsonNode.get("yaw").asDouble());
      pitch.set(jsonNode.get("pitch").asDouble());
      roll.set(jsonNode.get("roll").asDouble());
   }

   @Override
   public void performAction()
   {
      FrameYawPitchRoll frameChestYawPitchRoll = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getChestFrame());
      frameChestYawPitchRoll.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      frameChestYawPitchRoll.setYawPitchRoll(yaw.get(), pitch.get(), roll.get());
      frameChestYawPitchRoll.changeFrame(ReferenceFrame.getWorldFrame());

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(trajectoryTime.get(),
                                                                  frameChestYawPitchRoll,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  syncedRobot.getReferenceFrames().getPelvisZUpFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

      ros2ControllerHelper.publishToController(message);
   }
}
