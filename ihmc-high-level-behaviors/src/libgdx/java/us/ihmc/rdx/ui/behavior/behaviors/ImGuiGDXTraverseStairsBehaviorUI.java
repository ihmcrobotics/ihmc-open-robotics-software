package us.ihmc.rdx.ui.behavior.behaviors;

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
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.affordances.GDXBallAndArrowPosePlacement;
import us.ihmc.rdx.ui.behavior.registry.ImGuiGDXBehaviorUIDefinition;
import us.ihmc.rdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;
import us.ihmc.rdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.rdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.tools.Timer;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI.*;

public class ImGuiGDXTraverseStairsBehaviorUI extends ImGuiGDXBehaviorUIInterface
{
   public static final ImGuiGDXBehaviorUIDefinition DEFINITION = new ImGuiGDXBehaviorUIDefinition(TraverseStairsBehavior.DEFINITION,
                                                                                                  ImGuiGDXTraverseStairsBehaviorUI::new);

   private final BehaviorHelper helper;
   private final GDXFootstepPlanGraphic footstepPlanGraphic;
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final Stopwatch completedStopwatch = new Stopwatch();
   private String currentState = "";
   private String currentLifecycleState = "";
   private final ImBoolean operatorReviewEnabled = new ImBoolean(true);
   private final AtomicReference<Double> distanceToStairs;
   private final ImGuiEnumPlot currentLifecycleStatePlot = new ImGuiEnumPlot(1000, 250, 15);
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 15);
   private final ImGuiMovingPlot pauseTimeLeft = new ImGuiMovingPlot("Pause time left", 1000, 250, 15);
   private final ImGuiMovingPlot supportRegionsReceived = new ImGuiMovingPlot("Support regions received", 1000, 250, 15);
   private final ImGuiMovingPlot distanceToStairsPlot = new ImGuiMovingPlot("Distance to stairs", 1000, 250, 15);
   private long numberOfSupportRegionsReceived = 0;
   private final Timer supportRegionsReceivedTimer = new Timer();
   private final GDXBallAndArrowPosePlacement goalAffordance = new GDXBallAndArrowPosePlacement();
   private final ImGuiStoredPropertySetTuner footstepPlannerParameterTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (Stairs behavior)");
   private final ImGuiStoredPropertySetTuner swingPlannerParameterTuner = new ImGuiStoredPropertySetTuner("Swing Planner Parameters (Stairs behavior)");
   private double timeLeftInPause = 0.0;

   public ImGuiGDXTraverseStairsBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      footstepPlanGraphic = new GDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.PLANNED_STEPS, footsteps ->
      {
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps, DEFINITION.getName()));
      });
      footstepPlanGraphic.setTransparency(0.5);
      distanceToStairs = helper.subscribeViaReference(DistanceToStairs, Double.NaN);
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
      helper.subscribeViaCallback(PlanarRegionsForUI, regions ->
      {
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions);
      });
   }

   public void setGoal(Pose3D goal)
   {
      goalAffordance.setGoalPoseAndPassOn(goal);
   }

   @Override
   public void create(RDXBaseUI baseUI)
   {
      goalAffordance.create(goalPose -> helper.publish(GOAL_INPUT, goalPose), Color.SALMON);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGui3DViewInput);

      FootstepPlannerParametersBasics footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters("_Stairs");
      footstepPlannerParameterTuner.create(footstepPlannerParameters,
                                           () -> helper.publish(FootstepPlannerParameters, footstepPlannerParameters.getAllAsStrings()));
      SwingPlannerParametersBasics swingPlannerParameters = helper.getRobotModel().getSwingPlannerParameters("_Stairs");
      swingPlannerParameterTuner.create(swingPlannerParameters,
                                        () -> helper.publish(SwingPlannerParameters, swingPlannerParameters.getAllAsStrings()));
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      goalAffordance.processImGui3DViewInput(input);
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
   public void renderTreeNodeImGuiWidgets()
   {
      goalAffordance.renderPlaceGoalButton();
      ImGui.sameLine();
      ImGui.text(areGraphicsEnabled() ? "Showing graphics." : "Graphics hidden.");
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
      pauseTimeLeft.calculate(FormattingTools.getFormattedDecimal2D(timeLeftInPause));
      boolean supportRegionsReceivedRecently = supportRegionsReceivedTimer.isRunning(5.0);
      supportRegionsReceived.setNextValue(numberOfSupportRegionsReceived);
      supportRegionsReceived.calculate(supportRegionsReceivedRecently ? "DANGER" : "");
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
      distanceToStairsPlot.calculate(distanceToStairs.get().floatValue());
   }

   private void disableSupportRegions()
   {
      BipedalSupportPlanarRegionParametersMessage supportRegionParametersMessage = new BipedalSupportPlanarRegionParametersMessage();
      supportRegionParametersMessage.setEnable(false);
      helper.publish(ROS2Tools::getBipedalSupportRegionParametersTopic, supportRegionParametersMessage);
   }

   @Override
   public void addChildPanels(ImGuiPanel parentPanel)
   {
      parentPanel.addChild(footstepPlannerParameterTuner);
      parentPanel.addChild(swingPlannerParameterTuner);
   }

   @Override
   public void destroy()
   {
      footstepPlanGraphic.destroy();
      planarRegionsGraphic.destroy();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (areGraphicsEnabled())
      {
         footstepPlanGraphic.getRenderables(renderables, pool);
         goalAffordance.getRenderables(renderables, pool);
         planarRegionsGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
