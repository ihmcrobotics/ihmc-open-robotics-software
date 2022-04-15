package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

public class GDXChestOrientationAction implements GDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ROS2ControllerHelper ros2ControllerHelper;
   private ROS2SyncedRobotModel syncedRobot;
   private final ImBoolean selected = new ImBoolean();
   private final ImDouble yaw = new ImDouble();
   private final ImDouble pitch = new ImDouble();
   private final ImDouble roll = new ImDouble();
   private final ImDouble trajectoryTime = new ImDouble(4.0);

   public void create(ROS2ControllerHelper ros2ControllerHelper, ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
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
      ImGui.inputDouble(labels.get("Yaw"), yaw);
      ImGui.sameLine();
      ImGui.inputDouble(labels.get("Pitch"), pitch);
      ImGui.sameLine();
      ImGui.inputDouble(labels.get("Roll"), roll);
      ImGui.inputDouble(labels.get("Trajectory time"), trajectoryTime);
      ImGui.popItemWidth();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
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
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(2.0,
                                                                  frameChestYawPitchRoll,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  syncedRobot.getReferenceFrames().getPelvisZUpFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

      ros2ControllerHelper.publishToController(message);
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
      return "Chest Orientation";
   }
}
