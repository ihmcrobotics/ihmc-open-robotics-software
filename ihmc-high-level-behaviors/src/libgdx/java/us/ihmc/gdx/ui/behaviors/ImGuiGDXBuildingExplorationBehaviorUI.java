package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImDouble;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.behaviors.demo.BuildingExplorationBehavior;
import us.ihmc.behaviors.demo.BuildingExplorationBehaviorMode;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.*;

public class ImGuiGDXBuildingExplorationBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(BuildingExplorationBehavior.DEFINITION,
                                                                                        ImGuiGDXBuildingExplorationBehaviorUI::new);

   private final BehaviorHelper helper;
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();
   private final ImGuiGDXLookAndStepBehaviorUI lookAndStepUI;
   private final ImGuiGDXTraverseStairsBehaviorUI traverseStairsUI;
   private final Point2D nodePosition = new Point2D(319.0, 6.0);
   private final ImGuiGDXDoorBehaviorUI doorUI;
   private ImGuiLabelMap labels = new ImGuiLabelMap();
   private volatile BuildingExplorationBehaviorMode mode = BuildingExplorationBehaviorMode.AUTO;
   private final ImDouble distanceFromDoorToTransition = new ImDouble(1.8);

   public ImGuiGDXBuildingExplorationBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;

      lookAndStepUI = new ImGuiGDXLookAndStepBehaviorUI(helper);
      addChild(lookAndStepUI);
      traverseStairsUI = new ImGuiGDXTraverseStairsBehaviorUI(helper);
      addChild(traverseStairsUI);
      doorUI = new ImGuiGDXDoorBehaviorUI(helper);
      addChild(doorUI);

      helper.subscribeViaCallback(GoalForUI, goalAffordance::setGoalPose);
      helper.subscribeViaCallback(Mode, mode -> this.mode = mode);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      goalAffordance.create(baseUI, goalPose -> helper.publish(Goal, goalPose), Color.GREEN);
      baseUI.addImGui3DViewInputProcessor(goalAffordance::processImGui3DViewInput);
      baseUI.getSceneManager().addRenderableProvider(this, GDXSceneLevel.VIRTUAL);

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
   public void renderTreeNode()
   {
      goalAffordance.renderPlaceGoalButton();
      ImGui.pushItemWidth(150.0f);
      if (ImGui.inputDouble(labels.get("Distance from door to transition"), distanceFromDoorToTransition, 0.01, 0.5))
      {
         helper.publish(DistanceFromDoorToTransition, distanceFromDoorToTransition.get());
      }
      ImGui.popItemWidth();
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
   public void renderInternal()
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
