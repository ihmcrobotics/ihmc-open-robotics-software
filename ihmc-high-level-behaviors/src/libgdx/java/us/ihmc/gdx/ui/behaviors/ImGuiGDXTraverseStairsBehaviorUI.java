package us.ihmc.gdx.ui.behaviors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiEnumPlot;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.tools.Timer;

import static us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI.*;

public class ImGuiGDXTraverseStairsBehaviorUI extends GDXBehaviorUIInterface
{
   public static final GDXBehaviorUIDefinition DEFINITION = new GDXBehaviorUIDefinition(TraverseStairsBehavior.DEFINITION,
                                                                                        ImGuiGDXTraverseStairsBehaviorUI::new);

   private final BehaviorHelper helper;
   private final GDXFootstepPlanGraphic footstepPlanGraphic = new GDXFootstepPlanGraphic();
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private Point2D nodePosition = new Point2D(376.0, 517.0);
   private final Stopwatch completedStopwatch = new Stopwatch();
   private String currentState = "";
   private String currentLifecycleState = "";
   private final ImBoolean operatorReviewEnabled = new ImBoolean(true);
   private final ImGuiEnumPlot currentLifecycleStatePlot = new ImGuiEnumPlot(1000, 250, 15);
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 15);
   private final ImGuiMovingPlot pauseTimeLeft = new ImGuiMovingPlot("Pause time left", 1000, 250, 15);
   private final ImGuiMovingPlot supportRegionsReceived = new ImGuiMovingPlot("Support regions received", 1000, 250, 15);
   private long numberOfSupportRegionsReceived = 0;
   private final Timer supportRegionsReceivedTimer = new Timer();
   private final ImGuiGDXPoseGoalAffordance goalAffordance = new ImGuiGDXPoseGoalAffordance();
   private final ImGuiStoredPropertySetTuner footstepPlannerParameterTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (Stairs behavior)");
   private final ImGuiStoredPropertySetTuner swingPlannerParameterTuner = new ImGuiStoredPropertySetTuner("Swing Planner Parameters (Stairs behavior)");
   private double timeLeftInPause = 0.0;

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
      helper.subscribeViaCallback(LifecycleState, state -> currentLifecycleState = state);
      helper.subscribeViaCallback(TimeLeftInPause, timeLeftInPause -> this.timeLeftInPause = timeLeftInPause);
      helper.subscribeViaCallback(ROS2Tools.BIPEDAL_SUPPORT_REGIONS, regions ->
      {
         if (regions.getConvexPolygonsSize().size() > 0 && regions.getConvexPolygonsSize().get(0) > 0)
         {
            ++numberOfSupportRegionsReceived;
            supportRegionsReceivedTimer.reset();
         }
      });
   }

   public void setGoal(Pose3D goal)
   {
      goalAffordance.setGoalPose(goal);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      goalAffordance.create(baseUI, goalPose -> helper.publish(GOAL_INPUT, goalPose), Color.SALMON);
      baseUI.addImGui3DViewInputProcessor(this::processImGui3DViewInput);

      FootstepPlannerParametersBasics footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters("_Stairs");
      footstepPlannerParameterTuner.create(footstepPlannerParameters,
                                           FootstepPlannerParameterKeys.keys,
                                           () -> helper.publish(FootstepPlannerParameters, footstepPlannerParameters.getAllAsStrings()));
      SwingPlannerParametersBasics swingPlannerParameters = helper.getRobotModel().getSwingPlannerParameters("_Stairs");
      swingPlannerParameterTuner.create(swingPlannerParameters,
                                        SwingPlannerParameterKeys.keys,
                                        () -> helper.publish(SwingPlannerParameters, swingPlannerParameters.getAllAsStrings()));
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
      if (!currentLifecycleState.isEmpty())
      {
         TraverseStairsBehavior.TraverseStairsLifecycleStateName state = TraverseStairsBehavior.TraverseStairsLifecycleStateName.valueOf(currentLifecycleState);
         currentLifecycleStatePlot.render(state.ordinal(), state.name());
      }
      else
      {
         currentLifecycleStatePlot.render(-1, "");
      }
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
      pauseTimeLeft.setNextValue((float) timeLeftInPause);
      pauseTimeLeft.render(FormattingTools.getFormattedDecimal2D(timeLeftInPause));
      boolean supportRegionsReceivedRecently = supportRegionsReceivedTimer.isRunning(5.0);
      supportRegionsReceived.setNextValue(numberOfSupportRegionsReceived);
      supportRegionsReceived.render(supportRegionsReceivedRecently ? "DANGER" : "");
      if (ImGui.button("Disable support regions"))
      {
         disableSupportRegions();
      }
      ImGui.text("Completed: " + FormattingTools.getFormattedDecimal2D(completedStopwatch.totalElapsed()) + " s ago.");
      if (ImGui.checkbox(labels.get("Operator review"), operatorReviewEnabled))
      {
         helper.publish(OperatorReviewEnabled, operatorReviewEnabled.get());
      }
      if (ImGui.button(labels.get("Start")))
      {
         helper.publish(TraverseStairsBehaviorAPI.START);
         disableSupportRegions();
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

   private void disableSupportRegions()
   {
      BipedalSupportPlanarRegionParametersMessage supportRegionParametersMessage = new BipedalSupportPlanarRegionParametersMessage();
      supportRegionParametersMessage.setEnable(false);
      helper.publish(ROS2Tools::getBipedalSupportRegionParametersTopic, supportRegionParametersMessage);
   }

   @Override
   public void renderInternal()
   {
      footstepPlanGraphic.render();
      footstepPlannerParameterTuner.render();
      swingPlannerParameterTuner.render();
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
