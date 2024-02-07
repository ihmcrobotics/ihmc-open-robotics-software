package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import std_msgs.msg.dds.Empty;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Set;

public class RDXPerceptionVisualizerPanel extends RDXPanel implements RDXRenderableProvider
{
   private static final String WINDOW_NAME = "Perception Visualizers";

   private final LinkedHashMap<RDXVisualizer, RDXVisualizerWithHeartbeat> visualizers = new LinkedHashMap<>();
   private final ROS2Node heartbeatNode;

   private boolean created = false;

   public RDXPerceptionVisualizerPanel()
   {
      super(WINDOW_NAME);
      heartbeatNode = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "visualizer_hearbeat_node");
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void addVisualizer(RDXVisualizer visualizer)
   {
      addVisualizer(visualizer, null);
   }

   public void addVisualizer(RDXVisualizer visualizer, @Nullable ROS2Topic<Empty> visualizerHearbeatTopic, RDXVisualizer ... dependentVisualizers)
   {
      ArrayList<RDXVisualizerWithHeartbeat> dependentVisualizerWrappers = new ArrayList<>();
      for (RDXVisualizer dependentVisualizer : dependentVisualizers)
      {
         if (visualizers.get(dependentVisualizer) != null)
            dependentVisualizerWrappers.add(visualizers.get(dependentVisualizer));
      }

      RDXVisualizerWithHeartbeat wrappedVisualizer = new RDXVisualizerWithHeartbeat(heartbeatNode,
                                                                                    visualizerHearbeatTopic,
                                                                                    visualizer,
                                                                                    dependentVisualizerWrappers.toArray(new RDXVisualizerWithHeartbeat[0]));
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
         if (visualizer.getPanel() != null)
            visualizer.getPanel().getIsShowing().set(visualizer.isActive());
         if (visualizer.isActive())
            visualizer.update();
      }
   }

   public void renderImGuiWidgets()
   {
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
   }
}
