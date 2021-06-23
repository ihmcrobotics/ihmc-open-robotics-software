package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.behaviors.demo.BuildingExplorationBehavior;
import us.ihmc.behaviors.demo.BuildingExplorationBehaviorMode;
import us.ihmc.behaviors.demo.BuildingExplorationBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.*;

public class ImGuiGDXBuildingExplorationBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(BuildingExplorationBehavior.DEFINITION,
                                                                                        ImGuiGDXBuildingExplorationBehaviorUI::new);

   private final BehaviorHelper helper;
   private BuildingExplorationBehaviorParameters parameters;
   private final ImGuiStoredPropertySetTuner parameterTuner = new ImGuiStoredPropertySetTuner("Building Exploration Parameters");
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final ImGuiGDXTraverseStairsBehaviorUI traverseStairsUI;
   private final Point2D nodePosition = new Point2D(341.0, 5.0);
   private final ImGuiGDXDoorBehaviorUI doorUI;
   private ImGuiLabelMap labels = new ImGuiLabelMap();
   private volatile BuildingExplorationBehaviorMode mode = BuildingExplorationBehaviorMode.AUTO;

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
      helper.subscribeToPlanarRegionsViaCallback(ROS2Tools.LIDAR_REA_REGIONS, goalAffordance::setLatestRegions);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      parameters = new BuildingExplorationBehaviorParameters();
      parameterTuner.create(parameters, BuildingExplorationBehaviorParameters.keys, () -> helper.publish(Parameters, parameters.getAllAsStrings()));
      goalAffordance.create(baseUI, goalPose ->
      {
         helper.publish(Goal, goalPose);
         lookAndStepUI.setGoal(goalPose);
         traverseStairsUI.setGoal(goalPose);
      }, Color.GREEN);
      baseUI.addImGui3DViewInputProcessor(goalAffordance::processImGui3DViewInput);
      baseUI.get3DSceneManager().addRenderableProvider(this, GDXSceneLevel.VIRTUAL);

      lookAndStepUI.create(baseUI);
      traverseStairsUI.create(baseUI);
      doorUI.create(baseUI);
   }

   @Override
   public Point2D getTreeNodeInitialPosition()
   {
      return nodePosition;
   }

   @Override
   public void update()
   {

   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      goalAffordance.renderPlaceGoalButton();
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
   public void renderRegularPanelImGuiWidgets()
   {
   }

   @Override
   public void destroy()
   {
      lookAndStepUI.destroy();
      traverseStairsUI.destroy();
      doorUI.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
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
