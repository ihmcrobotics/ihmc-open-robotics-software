package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.behaviors.targetFollowing.TargetFollowingBehavior;
import us.ihmc.behaviors.targetFollowing.TargetFollowingBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;

import static us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorAPI.*;

public class ImGuiGDXTargetFollowingBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(TargetFollowingBehavior.DEFINITION,
                                                                                        ImGuiGDXTargetFollowingBehaviorUI::new);

   private final BehaviorHelper helper;
   private TargetFollowingBehaviorParameters parameters;
   private final ImGuiStoredPropertySetTuner parameterTuner = new ImGuiStoredPropertySetTuner("Target Following Parameters");
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private String lastTickedThing = "NONE";

   public ImGuiGDXTargetFollowingBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(helper);
      addChild(lookAndStepUI);
      helper.subscribeToPlanarRegionsViaCallback(ROS2Tools.MAPSENSE_REGIONS, regions ->
      {
         goalAffordance.setLatestRegions(regions);
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions);
      });
      helper.subscribeViaCallback(LastTickedThing, lastTickedThing -> this.lastTickedThing = lastTickedThing);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      parameters = new TargetFollowingBehaviorParameters();
      parameterTuner.create(parameters, TargetFollowingBehaviorParameters.keys, () -> helper.publish(Parameters, parameters.getAllAsStrings()));
      goalAffordance.create(baseUI, goalPose ->
      {
         helper.publish(Goal, goalPose);
         lookAndStepUI.setGoal(goalPose);
      }, Color.GREEN);
      baseUI.addImGui3DViewInputProcessor(goalAffordance::processImGui3DViewInput);
      lookAndStepUI.create(baseUI);
   }

   private boolean areGraphicsEnabled()
   {
      return wasTickedRecently(0.5) && lastTickedThing.equals("NONE");
   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      goalAffordance.renderPlaceGoalButton();
      ImGui.sameLine();
      ImGui.text(areGraphicsEnabled() ? "Showing graphics." : "Graphics hidden.");
      parameterTuner.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (areGraphicsEnabled())
      {
         planarRegionsGraphic.getRenderables(renderables, pool);
      }
      goalAffordance.getRenderables(renderables, pool);
      lookAndStepUI.getRenderables(renderables, pool);
   }

   @Override
   public void renderRegularPanelImGuiWidgets()
   {
   }

   @Override
   public void update()
   {
      if (areGraphicsEnabled())
      {
         planarRegionsGraphic.update();
      }
   }

   @Override
   public void destroy()
   {
      lookAndStepUI.destroy();
      planarRegionsGraphic.destroy();
   }
}
