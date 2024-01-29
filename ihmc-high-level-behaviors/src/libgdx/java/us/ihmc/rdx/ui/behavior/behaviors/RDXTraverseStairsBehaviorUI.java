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
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImGuiEnumPlot;
import us.ihmc.rdx.imgui.ImGuiLabelMap;
import us.ihmc.rdx.imgui.ImGuiMovingPlot;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXBallAndArrowPosePlacement;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.tools.Timer;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI.*;

public class RDXTraverseStairsBehaviorUI extends RDXBehaviorUIInterface
{
   private final BehaviorHelper helper;
   private final RDXFootstepPlanGraphic footstepPlanGraphic;
   private final RDXPlanarRegionsGraphic planarRegionsGraphic = new RDXPlanarRegionsGraphic();
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final Stopwatch completedStopwatch = new Stopwatch();
   private String currentState = "";
   private String currentLifecycleState = "";
   private final ImBoolean operatorReviewEnabled = new ImBoolean(true);
   private final AtomicReference<Double> distanceToStairs = null;
   private final ImGuiEnumPlot currentLifecycleStatePlot = new ImGuiEnumPlot(1000, 250, 15);
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 15);
   private final ImGuiMovingPlot pauseTimeLeft = new ImGuiMovingPlot("Pause time left", 1000, 250, 15);
   private final ImGuiMovingPlot supportRegionsReceived = new ImGuiMovingPlot("Support regions received", 1000, 250, 15);
   private final ImGuiMovingPlot distanceToStairsPlot = new ImGuiMovingPlot("Distance to stairs", 1000, 250, 15);
   private long numberOfSupportRegionsReceived = 0;
   private final Timer supportRegionsReceivedTimer = new Timer();
   private final RDXBallAndArrowPosePlacement goalAffordance = new RDXBallAndArrowPosePlacement();
   private final RDXStoredPropertySetTuner footstepPlannerParameterTuner = new RDXStoredPropertySetTuner("Footstep Planner Parameters (Stairs behavior)");
   private final RDXStoredPropertySetTuner swingPlannerParameterTuner = new RDXStoredPropertySetTuner("Swing Planner Parameters (Stairs behavior)");
   private double timeLeftInPause = 0.0;

   public RDXTraverseStairsBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      footstepPlanGraphic = new RDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.PLANNED_STEPS, footsteps ->
      {
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps, getDefinition().getDescription()));
      });
      footstepPlanGraphic.setOpacity(0.5);
//      distanceToStairs = helper.subscribeViaReference(DistanceToStairs, Double.NaN);
//      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.COMPLETED, completedStopwatch::reset);
//      helper.subscribeViaCallback(State, state -> currentState = state);
//      helper.subscribeViaCallback(LifecycleState, state -> currentLifecycleState = state);
//      helper.subscribeViaCallback(TimeLeftInPause, timeLeftInPause -> this.timeLeftInPause = timeLeftInPause);
      helper.subscribeViaCallback(PerceptionAPI.BIPEDAL_SUPPORT_REGIONS, regions ->
      {
         if (regions.getConvexPolygonsSize().size() > 0 && regions.getConvexPolygonsSize().get(0) > 0)
         {
            ++numberOfSupportRegionsReceived;
            supportRegionsReceivedTimer.reset();
         }
      });
//      helper.subscribeViaCallback(PlanarRegionsForUI, regions ->
//      {
//         if (regions != null)
//            planarRegionsGraphic.generateMeshesAsync(regions);
//      });
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
                                           () ->
                                           {
//                                              helper.publish(FootstepPlannerParameters, footstepPlannerParameters.getAllAsStrings());
                                           });
      SwingPlannerParametersBasics swingPlannerParameters = helper.getRobotModel().getSwingPlannerParameters("_Stairs");
      swingPlannerParameterTuner.create(swingPlannerParameters,
                                        () ->
                                        {
//                                           helper.publish(SwingPlannerParameters, swingPlannerParameters.getAllAsStrings());
                                        });
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
//         helper.publish(OperatorReviewEnabled, operatorReviewEnabled.get());
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
      helper.publish(PerceptionAPI::getBipedalSupportRegionParametersTopic, supportRegionParametersMessage);
   }

   @Override
   public void addChildPanels(RDXPanel parentPanel)
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
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (areGraphicsEnabled())
      {
         if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         {
            footstepPlanGraphic.getRenderables(renderables, pool);
            goalAffordance.getRenderables(renderables, pool);
         }
         if (sceneLevels.contains(RDXSceneLevel.MODEL))
         {
            planarRegionsGraphic.getRenderables(renderables, pool);
         }
      }
   }

   public String getName()
   {
      return "Traverse Stairs";
   }
}
