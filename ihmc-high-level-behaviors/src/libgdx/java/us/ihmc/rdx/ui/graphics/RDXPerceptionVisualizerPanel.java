package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import javax.annotation.Nullable;
import java.util.LinkedHashMap;
import java.util.Set;

public class RDXPerceptionVisualizerPanel extends RDXPanel implements RDXRenderableProvider
{
   private static final String WINDOW_NAME = "Perception Visualizers";

   private final LinkedHashMap<RDXVisualizer, RDXVisualizerWithHeartbeat> visualizers = new LinkedHashMap<>();
   private final ROS2Node heartbeatNode;

   private final ImBoolean removePointCloudOverlap = new ImBoolean(true);
   private final ROS2Heartbeat overlapRemovalHeartbeat;

   private boolean created = false;

   public RDXPerceptionVisualizerPanel()
   {
      super(WINDOW_NAME);
      heartbeatNode = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "visualizer_heartbeat_node");
      overlapRemovalHeartbeat = new ROS2Heartbeat(heartbeatNode, PerceptionAPI.REQUEST_OVERLAP_REMOVAL);
      overlapRemovalHeartbeat.setAlive(removePointCloudOverlap.get());
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void addVisualizer(RDXVisualizer visualizer)
   {
      addVisualizer(visualizer, null);
   }

   public void addVisualizer(RDXVisualizer visualizer, @Nullable ROS2Topic<Empty> visualizerHearbeatTopic)
   {
      RDXVisualizerWithHeartbeat wrappedVisualizer = new RDXVisualizerWithHeartbeat(heartbeatNode,
                                                                                    visualizerHearbeatTopic,
                                                                                    visualizer);
      visualizers.put(visualizer, wrappedVisualizer);
      RDXPanel panel = visualizer.getPanel();
      if (panel != null)
         addChild(panel);
      if (created)
         visualizer.create();
   }

   public void create()
   {
      for (RDXVisualizerWithHeartbeat visualizer : visualizers.values())
      {
         visualizer.create();
      }
      created = true;
   }

   public void update()
   {
      for (RDXVisualizerWithHeartbeat visualizer : visualizers.values())
      {
         if (visualizer.isActive())
            visualizer.update();
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.collapsingHeader("Visualizer Settings"))
      {
         if (ImGui.checkbox("Remove point cloud overlap", removePointCloudOverlap))
            overlapRemovalHeartbeat.setAlive(removePointCloudOverlap.get());
         ImGui.separator();
      }

      for (RDXVisualizerWithHeartbeat visualizer : visualizers.values())
      {
         visualizer.renderImGuiWidgets();
         ImGui.separator();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXVisualizerWithHeartbeat visualizer : visualizers.values())
      {
         if (visualizer.isActive())
         {
            visualizer.getRenderables(renderables, pool, sceneLevels);
         }
      }
   }

   public void destroy()
   {
      for (RDXVisualizerWithHeartbeat visualizer : visualizers.values())
      {
         visualizer.destroy();
      }

      overlapRemovalHeartbeat.destroy();
      heartbeatNode.destroy();
   }
}
