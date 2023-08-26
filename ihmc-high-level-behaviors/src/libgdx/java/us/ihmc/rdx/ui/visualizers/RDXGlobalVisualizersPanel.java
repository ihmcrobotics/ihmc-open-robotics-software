package us.ihmc.rdx.ui.visualizers;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.tools.Timer;

import java.util.ArrayList;
import java.util.Set;

public class RDXGlobalVisualizersPanel extends RDXPanel implements RDXRenderableProvider
{
   private static final String WINDOW_NAME = "Global Visualizers";

   private final ArrayList<RDXVisualizer> visualizers = new ArrayList<>();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private boolean created = false;
   private boolean firstFrame = true;
   private boolean startedTimer = false;
   private final Timer focusTimer = new Timer();

   public RDXGlobalVisualizersPanel()
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
      // This window is important, so we make sure it is showing on launch.
      // Unfortunately there's an ImGui issue where the docked windows show
      // the alphabetically last window on start instead of the one the user
      // had open when the configuration was saved.
      if (firstFrame)
      {
         firstFrame = false;

         startedTimer = true;
         focusTimer.reset();
      }
      if (startedTimer && focusTimer.isExpired(1.0))
      {
         startedTimer = false;
         ImGui.setWindowFocus(WINDOW_NAME);
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
   }
}
