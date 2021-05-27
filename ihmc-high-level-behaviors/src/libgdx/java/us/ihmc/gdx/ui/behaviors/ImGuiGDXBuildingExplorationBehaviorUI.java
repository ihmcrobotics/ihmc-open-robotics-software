package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.internal.ImGui;
import us.ihmc.behaviors.demo.BuildingExplorationBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.Goal;

public class ImGuiGDXBuildingExplorationBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(BuildingExplorationBehavior.DEFINITION,
                                                                                        ImGuiGDXBuildingExplorationBehaviorUI::new);

   private final BehaviorHelper helper;
   private final GDXFootstepPlanGraphic controllerFootsteps = new GDXFootstepPlanGraphic();
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final ImGuiGDXTraverseStairsBehaviorUI traverseStairsUI;

   public ImGuiGDXBuildingExplorationBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;

      helper.subscribeToControllerViaCallback(FootstepDataListMessage.class, footsteps ->
      {
         controllerFootsteps.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps));
      });

      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(helper);
      traverseStairsUI = new ImGuiGDXTraverseStairsBehaviorUI(helper);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      goalAffordance.create(baseUI, goalPose -> helper.publish(Goal, goalPose), Color.GREEN);

      lookAndStepUI.create(baseUI);
   }

   @Override
   public void render()
   {
      ImGui.text("Building Exploration");
      goalAffordance.renderPlaceGoalButton();

      int defaultOpen = ImGuiTreeNodeFlags.DefaultOpen;
      if (ImGui.collapsingHeader("Look and Step", defaultOpen))
      {
         lookAndStepUI.render();
      }

      controllerFootsteps.render();
   }

   @Override
   public void destroy()
   {
      controllerFootsteps.destroy();
      lookAndStepUI.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      goalAffordance.getRenderables(renderables, pool);
      controllerFootsteps.getRenderables(renderables, pool);
      lookAndStepUI.getRenderables(renderables, pool);
   }
}
