package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiEnumPlot;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.Goal;
import static us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI.*;

public class ImGuiGDXTraverseStairsBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(TraverseStairsBehavior.DEFINITION,
                                                                                        ImGuiGDXTraverseStairsBehaviorUI::new);

   private final BehaviorHelper helper;
   private final GDXFootstepPlanGraphic footstepPlanGraphic = new GDXFootstepPlanGraphic();
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private Point2D nodePosition = new Point2D(597.0, 529.0);
   private final Stopwatch completedStopwatch = new Stopwatch();
   private String currentState = "";
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 30);
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();

   public ImGuiGDXTraverseStairsBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.PLANNED_STEPS, footsteps ->
      {
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps));
      });
      footstepPlanGraphic.setTransparency(0.5);
      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.COMPLETED, completedStopwatch::reset);
      helper.subscribeViaCallback(State, state -> currentState = state);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      goalAffordance.create(baseUI, goalPose -> helper.publish(GOAL_INPUT, goalPose), Color.SALMON);
      helper.subscribeViaCallback(Goal, goalAffordance::setGoalPose);
      baseUI.addImGui3DViewInputProcessor(this::processImGui3DViewInput);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      goalAffordance.processImGui3DViewInput(input);
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
      ImGui.text("Current state:");
      if (!currentState.isEmpty())
      {
         TraverseStairsBehavior.TraverseStairsStateName state = TraverseStairsBehavior.TraverseStairsStateName.valueOf(currentState);
         currentStatePlot.render(state.ordinal(), state.name());
      }
      else
      {
         currentStatePlot.render(-1, "");
      }
      ImGui.text("Completed: " + FormattingTools.getFormattedDecimal2D(completedStopwatch.totalElapsed()) + " s ago.");
      if (ImGui.button(labels.get("Start")))
      {
         helper.publish(TraverseStairsBehaviorAPI.START);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Stop")))
      {
         helper.publish(TraverseStairsBehaviorAPI.STOP);
      }
      if (ImGui.button(labels.get("Execute steps")))
      {
         helper.publish(TraverseStairsBehaviorAPI.EXECUTE_STEPS);
      }
      if (ImGui.button(labels.get("Replan")))
      {
         helper.publish(TraverseStairsBehaviorAPI.REPLAN);
      }
   }

   @Override
   public void renderInternal()
   {
      footstepPlanGraphic.render();
   }

   @Override
   public void destroy()
   {
      footstepPlanGraphic.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepPlanGraphic.getRenderables(renderables, pool);
      goalAffordance.getRenderables(renderables, pool);
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
