package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXBallAndArrowPosePlacement;

import java.util.ArrayList;
import java.util.Set;

public class RDXGeneralToolsPanel extends RDXPanel implements RDXRenderableProvider
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final ArrayList<RDXVisualizer> visualizers = new ArrayList<>();
   private final RDXBaseUI baseUI;
   private final RDXBallAndArrowPosePlacement ballAndArrowPose = new RDXBallAndArrowPosePlacement();
   private boolean created = false;

   public RDXGeneralToolsPanel(RDXBaseUI baseUI)
   {
      super("General Tools");
      setRenderMethod(this::renderImGuiWidgets);

      this.baseUI = baseUI;
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

      ballAndArrowPose.create(null, Color.YELLOW);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ballAndArrowPose::processImGui3DViewInput);
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
      for (RDXVisualizer visualizer : visualizers)
      {
         visualizer.renderImGuiWidgets();
         ImGui.separator();
      }

      ballAndArrowPose.renderPlaceGoalButton();
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

      ballAndArrowPose.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      for (RDXVisualizer visualizer : visualizers)
      {
         visualizer.destroy();
      }
   }
}
