package us.ihmc.rdx.ui.behavior.behaviors;

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
import us.ihmc.rdx.imgui.ImGuiLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.affordances.RDXBallAndArrowPosePlacement;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIDefinition;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

import java.util.Set;

import static us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorAPI.*;

public class RDXBuildingExplorationBehaviorUI extends RDXBehaviorUIInterface
{
   public static final RDXBehaviorUIDefinition DEFINITION = new RDXBehaviorUIDefinition(BuildingExplorationBehavior.DEFINITION,
                                                                                        RDXBuildingExplorationBehaviorUI::new);

   private final BehaviorHelper helper;
   private BuildingExplorationBehaviorParameters parameters;
   private final ImGuiStoredPropertySetTuner parameterTuner = new ImGuiStoredPropertySetTuner("Building Exploration Parameters");
   private final RDXBallAndArrowPosePlacement goalAffordance = new RDXBallAndArrowPosePlacement();
   private final RDXLookAndStepBehaviorUI lookAndStepUI;
   private final RDXTraverseStairsBehaviorUI traverseStairsUI;
   private final RDXDoorBehaviorUI doorUI;
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private volatile BuildingExplorationBehaviorMode mode = BuildingExplorationBehaviorMode.TELEOP;
   private final RDXPlanarRegionsGraphic planarRegionsGraphic = new RDXPlanarRegionsGraphic();
   private String lastTickedThing = "NONE";

   public RDXBuildingExplorationBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;

      lookAndStepUI = new RDXLookAndStepBehaviorUI(helper);
      addChild(lookAndStepUI);
      doorUI = new RDXDoorBehaviorUI(helper);
      addChild(doorUI);
      traverseStairsUI = new RDXTraverseStairsBehaviorUI(helper);
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
   public void create(RDXBaseUI baseUI)
   {
      parameters = new BuildingExplorationBehaviorParameters();
      parameterTuner.create(parameters, () -> helper.publish(Parameters, parameters.getAllAsStrings()));
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
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (areGraphicsEnabled() && sceneLevels.contains(RDXSceneLevel.MODEL))
      {
         planarRegionsGraphic.getRenderables(renderables, pool);
      }
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         goalAffordance.getRenderables(renderables, pool);
      }
      lookAndStepUI.getRenderables(renderables, pool, sceneLevels);
      traverseStairsUI.getRenderables(renderables, pool, sceneLevels);
      doorUI.getRenderables(renderables, pool, sceneLevels);
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
