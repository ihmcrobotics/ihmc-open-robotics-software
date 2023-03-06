package us.ihmc.rdx.ui.visualizers;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.utilities.ros.ROS1Helper;

import java.util.ArrayList;
import java.util.Set;

public class RDXGlobalVisualizersPanel extends ImGuiPanel implements RDXRenderableProvider
{
   private static final String WINDOW_NAME = "Global Visualizers";

   private final ArrayList<RDXVisualizer> visualizers = new ArrayList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS1Helper ros1Helper;
   private boolean created = false;

   public RDXGlobalVisualizersPanel()
   {
      this(new ROS1Helper("global_visualizers"));
   }

   public RDXGlobalVisualizersPanel(boolean enableROS1)
   {
      this(enableROS1 ? new ROS1Helper("global_visualizers") : null);
   }

   public RDXGlobalVisualizersPanel(ROS1Helper ros1Helper)
   {
      super(WINDOW_NAME);
      this.ros1Helper = ros1Helper;
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void addVisualizer(RDXVisualizer visualizer)
   {
      visualizers.add(visualizer);
      ImGuiPanel panel = visualizer.getPanel();
      if (panel != null)
         addChild(panel);
      if (created)
         visualizer.create();
   }

   public void create()
   {
      created = true;
      for (RDXVisualizer visualizer : visualizers)
      {
         visualizer.create();
      }
   }

   public void update()
   {
      for (RDXVisualizer visualizer : visualizers)
      {
         if (visualizer instanceof RDXROS1VisualizerInterface && ros1Helper != null)
         {
            ((RDXROS1VisualizerInterface) visualizer).updateSubscribers(ros1Helper);
         }
         if (visualizer.getPanel() != null)
            visualizer.getPanel().getIsShowing().set(visualizer.isActive());
         if (visualizer.isActive())
         {
            visualizer.update();
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (ros1Helper != null && ImGui.button(labels.get("Reconnect ROS 1 Node")))
      {
         ros1Helper.reconnectEverything();
      }
      for (RDXVisualizer visualizer : visualizers)
      {
         visualizer.renderImGuiWidgets();
         ImGui.separator();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXVisualizer visualizer : visualizers)
      {
         if (visualizer.isActive())
         {
            visualizer.getRenderables(renderables, pool, sceneLevels);
         }
      }
   }

   public void destroy()
   {
      for (RDXVisualizer visualizer : visualizers)
      {
         visualizer.destroy();
      }
      if (ros1Helper != null)
         ros1Helper.destroy();
   }
}