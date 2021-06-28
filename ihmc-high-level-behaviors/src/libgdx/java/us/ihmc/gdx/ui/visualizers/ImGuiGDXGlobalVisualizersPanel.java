package us.ihmc.gdx.ui.visualizers;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.utilities.ros.ROS1Helper;

import java.util.ArrayList;

public class ImGuiGDXGlobalVisualizersPanel extends ImGuiPanel implements RenderableProvider
{
   private static final String WINDOW_NAME = "Global Visualizers";

   private final ArrayList<ImGuiGDXVisualizer> visualizers = new ArrayList<>();

   private final ROS1Helper ros1Helper = new ROS1Helper("global_visualizers");

   public ImGuiGDXGlobalVisualizersPanel()
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void addVisualizer(ImGuiGDXVisualizer visualizer)
   {
      visualizers.add(visualizer);
      ImGuiPanel panel = visualizer.getPanel();
      if (panel != null)
         addChild(panel);
   }

   public void create()
   {
      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         visualizer.create();
      }
   }

   public void update()
   {
      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         if (visualizer instanceof ImGuiGDXROS1Visualizer)
         {
            ((ImGuiGDXROS1Visualizer) visualizer).updateSubscribers(ros1Helper);
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
      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         visualizer.renderImGuiWidgets();
         ImGui.separator();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         if (visualizer.isActive())
         {
            visualizer.getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      for (ImGuiGDXVisualizer visualizer : visualizers)
      {
         visualizer.destroy();
      }
      ros1Helper.destroy();
   }
}