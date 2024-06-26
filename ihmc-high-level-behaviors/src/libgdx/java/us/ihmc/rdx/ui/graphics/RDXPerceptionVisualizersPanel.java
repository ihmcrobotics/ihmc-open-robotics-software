package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImString;
import rosgraph_msgs.Log;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Comparator;
import java.util.Locale;
import java.util.Set;
import java.util.TreeSet;

public class RDXPerceptionVisualizersPanel extends RDXPanel implements RDXRenderableProvider
{
   private static final String WINDOW_NAME = "Perception Visualizers";

   private final TreeSet<RDXVisualizer> visualizers = new TreeSet<>(Comparator.comparing(RDXVisualizer::getTitle));
   // Search filter
   private final ImString filter = new ImString();
   private boolean created = false;

   public RDXPerceptionVisualizersPanel()
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void addVisualizer(RDXVisualizer visualizer)
   {
      visualizers.add(visualizer);
      RDXPanel panel = visualizer.getPanel();
      if (panel != null)
         addChild(panel);
      if (created)
         visualizer.create();
   }

   public void create()
   {
      for (RDXVisualizer visualizer : visualizers)
      {
         visualizer.create();
      }
      created = true;
   }

   public void update()
   {
      for (RDXVisualizer visualizer : visualizers)
      {
         if (visualizer.isActive())
            visualizer.update();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.inputText("Search##perceptionVisualizers", filter);
      ImGui.sameLine();
      boolean collapse = ImGui.button("Collapse##perceptionVisualizers");

      double longestVisualizerNameLength = 0.0;
      for (RDXVisualizer visualizer : visualizers)
      {
         double visualizerNameLength = ImGuiTools.calcTextSizeX(visualizer.getTitle());
         if (visualizerNameLength > longestVisualizerNameLength)
            longestVisualizerNameLength = visualizerNameLength;
      }
      boolean renderRightContext = (ImGui.getWindowSizeX() - 120) > longestVisualizerNameLength;

      // Pinned visualizers
      ImGui.separator();
      for (RDXVisualizer visualizer : visualizers)
         if (visualizer.isPinned())
            visualizer.renderMenuEntry(collapse, renderRightContext);
      ImGui.separator();

      // All other visualizers in alphabetical order (filtered)
      for (RDXVisualizer visualizer : visualizers)
      {
         if (!visualizer.isPinned())
         {
            if (filter.isNotEmpty())
               if (!visualizer.getTitle().toLowerCase(Locale.ROOT).contains(filter.toString().toLowerCase(Locale.ROOT)))
                  continue;

            visualizer.renderMenuEntry(collapse, renderRightContext);
         }
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
         visualizer.destroy();

      visualizers.clear();
   }
}
