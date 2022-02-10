package us.ihmc.gdx.ui.behavior.behaviors;

import com.badlogic.gdx.graphics.Color;
import imgui.internal.ImGui;
import us.ihmc.behaviors.heightMapNavigation.HeightMapNavigationBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;

import static us.ihmc.behaviors.heightMapNavigation.HeightMapNavigationBehaviorAPI.GoalPose;
import static us.ihmc.behaviors.heightMapNavigation.HeightMapNavigationBehaviorAPI.PlanarRegionsForUI;

public class ImGuiGDXHeightMapNavigationBehaviorUI extends ImGuiGDXBehaviorUIInterface
{
   public static final ImGuiGDXBehaviorUIDefinition DEFINITION = new ImGuiGDXBehaviorUIDefinition(HeightMapNavigationBehavior.DEFINITION,
                                                                                                  ImGuiGDXHeightMapNavigationBehaviorUI::new);

   private final BehaviorHelper helper;
   private final GDXFootstepPlanGraphic footstepPlanGraphic = new GDXFootstepPlanGraphic();
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();

   public ImGuiGDXHeightMapNavigationBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;

      helper.subscribeViaCallback(PlanarRegionsForUI, regions ->
      {
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions);
      });
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      goalAffordance.create(baseUI, goalPose -> helper.publish(GoalPose, goalPose), Color.GREEN);
      baseUI.addImGui3DViewInputProcessor(goalAffordance::processImGui3DViewInput);
   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      goalAffordance.renderPlaceGoalButton();
      ImGui.sameLine();
      ImGui.text(areGraphicsEnabled() ? "Showing graphics." : "Graphics hidden.");
   }

   @Override
   public void update()
   {
      if (areGraphicsEnabled())
      {
         footstepPlanGraphic.update();
         planarRegionsGraphic.update();
      }
   }

   private boolean areGraphicsEnabled()
   {
      return wasTickedRecently(0.5);
   }

   @Override
   public void destroy()
   {
      footstepPlanGraphic.destroy();
      planarRegionsGraphic.destroy();
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}

