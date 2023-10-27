package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.perception.CameraModel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import javax.annotation.Nullable;
import java.util.Set;

public class RDXVisualizerWrapper
{
   private final RDXVisualizer visualizer;
   @Nullable
   private ROS2Heartbeat heartbeat = null;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean isAlive = new ImBoolean(false);

   public RDXVisualizerWrapper(ROS2Node node, @Nullable ROS2Topic<Empty> heartbeatTopic, RDXVisualizer visualizer)
   {
      this.visualizer = visualizer;
      if (heartbeatTopic != null)
         heartbeat = new ROS2Heartbeat(node, heartbeatTopic);
   }

   public void create()
   {
      visualizer.create();
   }

   public void renderImGuiWidgets()
   {
      if (visualizer instanceof RDXROS2ImageMessageVisualizer || visualizer instanceof RDXROS2ColoredPointCloudVisualizer)
      {
         if (ImGui.checkbox(labels.get(visualizer.getTitle()), isAlive))
         {
            visualizer.setActive(isAlive.get());
            if (heartbeat != null)
               heartbeat.setAlive(isAlive());

            if (visualizer instanceof RDXROS2ImageMessageVisualizer)
            {
               ((RDXROS2ImageMessageVisualizer) visualizer).setSubscribed(isAlive.get());
            }
            else if (visualizer instanceof RDXROS2ColoredPointCloudVisualizer)
            {
               ((RDXROS2ColoredPointCloudVisualizer) visualizer).setSubscribed(isAlive.get());
            }
         }
         ImGuiTools.previousWidgetTooltip("Active");

         renderImGuiWidgetsByType();
      }
      else
      {
         visualizer.renderImGuiWidgets();
      }
   }

   public void update()
   {
      visualizer.update();
   }

   public RDXPanel getPanel()
   {
      return visualizer.getPanel();
   }

   public boolean isAlive()
   {
      if (visualizer instanceof RDXROS2ImageMessageVisualizer || visualizer instanceof RDXROS2ColoredPointCloudVisualizer)
         return isAlive.get();
      else
         return visualizer.isActive();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      visualizer.getRenderables(renderables, pool, sceneLevels);
   }

   public void destroy()
   {
      visualizer.destroy();
      if (heartbeat != null)
         heartbeat.destroy();
   }

   private void renderImGuiWidgetsByType()
   {
      if (visualizer instanceof RDXROS2ImageMessageVisualizer)
      {
         RDXROS2ImageMessageVisualizer imageMessageVisualizer = (RDXROS2ImageMessageVisualizer) visualizer;

         ImGui.text(imageMessageVisualizer.getTitle());
         if (imageMessageVisualizer.getHasReceivedOne())
         {
            imageMessageVisualizer.renderStatistics();
         }
      }
      else if (visualizer instanceof RDXROS2ColoredPointCloudVisualizer)
      {
         RDXROS2ColoredPointCloudVisualizer pointCloudVisualizer = (RDXROS2ColoredPointCloudVisualizer) visualizer;

         ImGui.text(pointCloudVisualizer.getTitle());

         pointCloudVisualizer.renderStatistics();

         ImGui.checkbox(labels.get("Use sensor color"), pointCloudVisualizer.useSensorColor());
         ImGui.text("Gradient mode:");
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("World Z"), pointCloudVisualizer.getGradientMode() == RDXColorGradientMode.WORLD_Z))
            pointCloudVisualizer.setGradientMode(RDXColorGradientMode.WORLD_Z);
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Sensor X"), pointCloudVisualizer.getGradientMode() == RDXColorGradientMode.SENSOR_X))
            pointCloudVisualizer.setGradientMode(RDXColorGradientMode.SENSOR_X);
         ImGui.checkbox(labels.get("Sinusoidal gradient"), pointCloudVisualizer.useSinusoidalGradientPattern());
         ImGui.sliderFloat(labels.get("Point scale"), pointCloudVisualizer.getPointSizeScale().getData(), 0.0f, 2.0f);
         if (pointCloudVisualizer.getDepthChannelCamera() == CameraModel.OUSTER && pointCloudVisualizer.getColorChannelCamera() == CameraModel.EQUIDISTANT_FISHEYE)
         {
            ImGui.sliderInt(labels.get("Level of color detail"), pointCloudVisualizer.getLevelOfColorDetail().getData(), 0, 3);
         }
      }
   }
}
