package us.ihmc.gdx.ui.behavior.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.behaviors.heightMapNavigation.HeightMapNavigationBehavior;
import us.ihmc.behaviors.heightMapNavigation.HeightMapNavigationBehaviorAPI;
import us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.visualizers.GDXHeightMapGraphic;
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
   private final GDXHeightMapGraphic heightMapGraphic = new GDXHeightMapGraphic();

   public ImGuiGDXHeightMapNavigationBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;

      helper.subscribeViaCallback(PlanarRegionsForUI, regions ->
      {
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions);
      });

      helper.subscribeViaCallback(ROS2Tools.HEIGHT_MAP_OUTPUT, heightMapGraphic::generateMeshesAsync);
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

      if (ImGui.button("Start"))
      {
         helper.getMessager().submitMessage(HeightMapNavigationBehaviorAPI.RequestStart, true);
      }
   }

   @Override
   public void update()
   {
      if (areGraphicsEnabled())
      {
         footstepPlanGraphic.update();
         planarRegionsGraphic.update();
         heightMapGraphic.update();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      goalAffordance.getRenderables(renderables, pool);
      heightMapGraphic.getRenderables(renderables, pool);
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
      heightMapGraphic.destroy();
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}

