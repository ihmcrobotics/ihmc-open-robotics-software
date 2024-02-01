package us.ihmc.rdx.ui.behavior.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import org.apache.commons.lang3.StringUtils;
import std_msgs.msg.dds.UInt16;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorMode;
import us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.rdx.imgui.ImGuiLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.affordances.RDXBallAndArrowPosePlacement;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;

import java.util.Set;

import static us.ihmc.behaviors.buildingExploration.BuildingExplorationBehaviorAPI.*;

public class RDXBuildingExplorationBehaviorUI extends RDXBehaviorUIInterface
{
   private final BehaviorHelper helper;
   private BuildingExplorationBehaviorParameters parameters;
   private final RDXStoredPropertySetTuner parameterTuner = new RDXStoredPropertySetTuner("Building Exploration Parameters");
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

      helper.subscribeViaCallback(MODE, message -> this.mode = BuildingExplorationBehaviorMode.values()[message.getData()]);
      helper.subscribeToPlanarRegionsViaCallback(PerceptionAPI.LIDAR_REA_REGIONS, regions ->
      {
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions);
      });
      helper.subscribeViaCallback(LAST_TICKED_NODE, lastTickedNode -> this.lastTickedThing = lastTickedNode.getDataAsString());
   }

   @Override
   public void create(RDXBaseUI baseUI)
   {
      parameters = new BuildingExplorationBehaviorParameters();
      parameterTuner.create(parameters, () ->
            helper.publish(PARAMETERS.getCommandTopic(), StoredPropertySetMessageTools.newMessage(parameters)));
      goalAffordance.create(goalPose ->
      {
         helper.publish(GOAL_COMMAND, goalPose);
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
      return getState().getIsActive();
   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      ImGui.text("Goal Planning");
      ImGui.sameLine();
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
            UInt16 modeMessage = new UInt16();
            modeMessage.setData(modeValue.ordinal());
            helper.publish(MODE, modeMessage);
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

   public String getName()
   {
      return "Building Exploration";
   }
}
