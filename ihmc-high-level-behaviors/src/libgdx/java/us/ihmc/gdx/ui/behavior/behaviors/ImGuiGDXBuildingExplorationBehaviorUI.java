package us.ihmc.gdx.ui.behavior.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationBehavior;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorMode;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.GDXBallAndArrowPosePlacement;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;

import static us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorAPI.*;

public class ImGuiGDXBuildingExplorationBehaviorUI extends ImGuiGDXBehaviorUIInterface
{
   public static final ImGuiGDXBehaviorUIDefinition DEFINITION = new ImGuiGDXBehaviorUIDefinition(BuildingExplorationBehavior.DEFINITION,
                                                                                                  ImGuiGDXBuildingExplorationBehaviorUI::new);

   private final BehaviorHelper helper;
   private BuildingExplorationBehaviorParameters parameters;
   private final ImGuiStoredPropertySetTuner parameterTuner = new ImGuiStoredPropertySetTuner("Building Exploration Parameters");
   private final GDXBallAndArrowPosePlacement goalAffordance = new GDXBallAndArrowPosePlacement();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final ImGuiGDXTraverseStairsBehaviorUI traverseStairsUI;
   private final ImGuiGDXDoorBehaviorUI doorUI;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private volatile BuildingExplorationBehaviorMode mode = BuildingExplorationBehaviorMode.TELEOP;
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private String lastTickedThing = "NONE";

   public ImGuiGDXBuildingExplorationBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;

      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(helper);
      addChild(lookAndStepUI);
      doorUI = new ImGuiGDXDoorBehaviorUI(helper);
      addChild(doorUI);
      traverseStairsUI = new ImGuiGDXTraverseStairsBehaviorUI(helper);
      addChild(traverseStairsUI);

      helper.subscribeViaCallback(Mode, mode -> this.mode = mode);
      helper.subscribeToPlanarRegionsViaCallback(ROS2Tools.LIDAR_REA_REGIONS, regions ->
      {
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions);
      });
      helper.subscribeViaCallback(LastTickedThing, lastTickedThing -> this.lastTickedThing = lastTickedThing);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      parameters = new BuildingExplorationBehaviorParameters();
      parameterTuner.create(parameters, BuildingExplorationBehaviorParameters.keys, () -> helper.publish(Parameters, parameters.getAllAsStrings()));
      goalAffordance.create(goalPose ->
      {
         helper.publish(Goal, goalPose);
         lookAndStepUI.setGoal(goalPose);
         traverseStairsUI.setGoal(goalPose);
      }, Color.GREEN);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(goalAffordance::processImGui3DViewInput);

      lookAndStepUI.create(baseUI);
      traverseStairsUI.create(baseUI);
      doorUI.create(baseUI);
   }

   @Override
   public void update()
   {
      if (areGraphicsEnabled())
      {
         planarRegionsGraphic.update();
      }
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
      ImGui.text("Mode:");
      for (BuildingExplorationBehaviorMode modeValue : BuildingExplorationBehaviorMode.values())
      {
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get(StringUtils.capitalize(modeValue.name().toLowerCase().replaceAll("_", " "))), mode.equals(modeValue)))
         {
            helper.publish(Mode, modeValue);
         }
      }
   }

   @Override
   public void destroy()
   {
      lookAndStepUI.destroy();
      traverseStairsUI.destroy();
      doorUI.destroy();
      planarRegionsGraphic.destroy();
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
      traverseStairsUI.getRenderables(renderables, pool);
      doorUI.getRenderables(renderables, pool);
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
