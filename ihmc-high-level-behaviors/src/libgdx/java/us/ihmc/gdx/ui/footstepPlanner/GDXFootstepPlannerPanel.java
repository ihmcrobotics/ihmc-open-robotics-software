package us.ihmc.gdx.ui.footstepPlanner;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXFootstepPlannerGoalGizmo;

public class GDXFootstepPlannerPanel extends ImGuiPanel implements RenderableProvider
{
   private final GDXFootstepPlannerGoalGizmo goalGizmo = new GDXFootstepPlannerGoalGizmo(getClass().getSimpleName());

   public GDXFootstepPlannerPanel()
   {
      super("Footstep Planner");
      setRenderMethod(this::renderImGuiWidgets);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      goalGizmo.create(baseUI.get3DSceneManager().getCamera3D());
      baseUI.addImGui3DViewInputProcessor(goalGizmo::process3DViewInput);
      baseUI.get3DSceneManager().addRenderableProvider(this, GDXSceneLevel.VIRTUAL);
   }

   private void renderImGuiWidgets()
   {
      if (ImGui.button("Place"))
      {

      }

      goalGizmo.renderImGuiTuner();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      goalGizmo.getRenderables(renderables, pool);
   }
}
