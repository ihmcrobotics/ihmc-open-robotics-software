package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

public class GDXPelvisHeightAction implements GDXBehaviorAction
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ROS2ControllerHelper ros2ControllerHelper;
   private final ImBoolean selected = new ImBoolean();
   private final ImDouble heightInWorld = new ImDouble();
   private final ImDouble trajectoryTime = new ImDouble(4.0);

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
      ImGui.pushItemWidth(80.0f);
      ImGui.inputDouble(labels.get("Height in world"), heightInWorld);
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
      jsonNode.put("heightInWorld", heightInWorld.get());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      trajectoryTime.set(jsonNode.get("trajectoryTime").asDouble());
      heightInWorld.set(jsonNode.get("heightInWorld").asDouble());
   }

   @Override
   public void performAction()
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory()
             .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime.get(),
                                                                        new Point3D(0.0, 0.0, heightInWorld.get()),
                                                                        ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);

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
      return "Pelvis Height";
   }
}
