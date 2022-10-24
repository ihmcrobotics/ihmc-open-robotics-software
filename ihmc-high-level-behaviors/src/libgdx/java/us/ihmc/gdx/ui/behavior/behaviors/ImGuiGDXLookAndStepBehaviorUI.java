package us.ihmc.gdx.ui.behavior.behaviors;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import com.badlogic.gdx.graphics.*;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.gdx.imgui.*;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.GDXBallAndArrowPosePlacement;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIDefinition;
import us.ihmc.gdx.ui.behavior.registry.ImGuiGDXBehaviorUIInterface;
import us.ihmc.gdx.ui.graphics.GDXBodyPathPlanGraphic;
import us.ihmc.gdx.ui.graphics.GDXBoxVisualizer;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.ui.yo.ImGuiYoDoublePlot;
import us.ihmc.gdx.ui.yo.ImPlotYoHelperDoublePlotLine;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.gdx.visualizers.GDXSphereAndArrowGraphic;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class ImGuiGDXLookAndStepBehaviorUI extends ImGuiGDXBehaviorUIInterface
{
   public static final ImGuiGDXBehaviorUIDefinition DEFINITION = new ImGuiGDXBehaviorUIDefinition(LookAndStepBehavior.DEFINITION,
                                                                                                  ImGuiGDXLookAndStepBehaviorUI::new);

   private final BehaviorHelper helper;
   private final AtomicReference<ArrayList<MinimalFootstep>> latestPlannedFootsteps;
   private final AtomicReference<ArrayList<MinimalFootstep>> latestCommandedFootsteps;
   private String currentState = "";
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean operatorReview = new ImBoolean(true);
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 15);
   private long numberOfSteppingRegionsReceived = 0;
   private final ImGuiPlot steppingRegionsPlot = new ImGuiPlot("", 1000, 250, 15);
   private final ImGuiMovingPlot impassibilityDetectedPlot = new ImGuiMovingPlot("Impassibility", 1000, 250, 15);
   private final AtomicReference<Boolean> impassibilityDetected;
   private final ImBoolean stopForImpassibilities = new ImBoolean(true);
   private final ImPlotYoHelperDoublePlotLine footstepPlanningDurationPlot;
   private final ImGuiYoDoublePlot footholdVolumePlot;

   private boolean reviewingBodyPath = true;
   private final ImString latestFootstepPlannerLogPath = new ImString();
   private ArrayList<Pair<Integer, Double>> latestFootstepPlannerRejectionReasons = new ArrayList<>();

   private final GDXSphereAndArrowGraphic subGoalGraphic = new GDXSphereAndArrowGraphic();
   private final GDXPlanarRegionsGraphic planarRegionsGraphic = new GDXPlanarRegionsGraphic();
   private final GDXPlanarRegionsGraphic receivedRegionsGraphic = new GDXPlanarRegionsGraphic();
   private final GDXBodyPathPlanGraphic bodyPathPlanGraphic = new GDXBodyPathPlanGraphic();
   private final GDXFootstepPlanGraphic footstepPlanGraphic;
   private final GDXFootstepPlanGraphic commandedFootstepsGraphic;
   private final GDXFootstepPlanGraphic startAndGoalFootstepsGraphic;
   private final ImGuiStoredPropertySetTuner lookAndStepParameterTuner = new ImGuiStoredPropertySetTuner("Look and Step Parameters");
   private final ImGuiStoredPropertySetTuner footstepPlannerParameterTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (for Look and Step)");
   private final ImGuiStoredPropertySetTuner swingPlannerParameterTuner = new ImGuiStoredPropertySetTuner("Swing Planner Parameters (for Look and Step)");
   private final GDXBallAndArrowPosePlacement goalAffordance = new GDXBallAndArrowPosePlacement();
   private final GDXBoxVisualizer obstacleBoxVisualizer = new GDXBoxVisualizer();
   private final ImBoolean invertShowGraphics = new ImBoolean(false);
   private final ImBoolean showReceivedRegions = new ImBoolean(false);

   public ImGuiGDXLookAndStepBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      helper.subscribeViaCallback(CurrentState, state -> currentState = state);
      helper.subscribeViaCallback(OperatorReviewEnabledToUI, operatorReview::set);
      helper.subscribeViaCallback(PlanarRegionsForUI, regions ->
      {
         ++numberOfSteppingRegionsReceived;
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(regions.copy());
      });
      helper.subscribeViaCallback(ReceivedPlanarRegionsForUI, regions ->
      {
         if (regions != null)
            receivedRegionsGraphic.generateMeshesAsync(regions.copy());
      });
      helper.subscribeViaCallback(GoalForUI, goalAffordance::setGoalPoseNoCallbacks);
      helper.subscribeViaCallback(SubGoalForUI, subGoalGraphic::setToPose);
      helper.subscribeViaCallback(BodyPathPlanForUI, bodyPath ->
      {
         if (bodyPath != null)
            Gdx.app.postRunnable(() -> bodyPathPlanGraphic.generateMeshes(bodyPath));
      });
      footstepPlanGraphic = new GDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      commandedFootstepsGraphic = new GDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      startAndGoalFootstepsGraphic = new GDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      footstepPlanGraphic.setTransparency(0.2);
      latestPlannedFootsteps = helper.subscribeViaReference(PlannedFootstepsForUI, new ArrayList<>());
      latestCommandedFootsteps = helper.subscribeViaReference(LastCommandedFootsteps, new ArrayList<>());
      helper.subscribeViaCallback(PlannedFootstepsForUI, footsteps ->
      {
         reviewingBodyPath = false;
         footstepPlanGraphic.generateMeshesAsync(footsteps);
      });
      helper.subscribeViaCallback(LastCommandedFootsteps, commandedFootstepsGraphic::generateMeshesAsync);
      startAndGoalFootstepsGraphic.setColor(RobotSide.LEFT, Color.BLUE);
      startAndGoalFootstepsGraphic.setColor(RobotSide.RIGHT, Color.BLUE);
      startAndGoalFootstepsGraphic.setTransparency(0.4);
      helper.subscribeViaCallback(ImminentFootPosesForUI, startAndGoalFootstepsGraphic::generateMeshesAsync);
      footstepPlanningDurationPlot = new ImPlotYoHelperDoublePlotLine("LookAndStepBehavior.footstepPlanningDuration", 10.0, helper);
      helper.subscribeViaCallback(FootstepPlannerLatestLogPath, latestFootstepPlannerLogPath::set);
      helper.subscribeViaCallback(FootstepPlannerRejectionReasons, reasons -> latestFootstepPlannerRejectionReasons = reasons);
      footholdVolumePlot = new ImGuiYoDoublePlot("footholdVolume", helper, 1000, 250, 15);
      impassibilityDetected = helper.subscribeViaReference(ImpassibilityDetected, false);
      obstacleBoxVisualizer.setColor(Color.RED);
      helper.subscribeViaCallback(Obstacle, boxDescription ->
      {
         Box3D box3D = new Box3D();
         box3D.set(boxDescription.getLeft(), boxDescription.getRight());
         obstacleBoxVisualizer.generateMeshAsync(box3D);
      });
      helper.subscribeViaCallback(ResetForUI, goalAffordance::clear);
   }

   @Override
   public void create(GDXImGuiBasedUI baseUI)
   {
      LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();
      lookAndStepParameterTuner.create(lookAndStepParameters,
                                       LookAndStepBehaviorParameters.keys,
                                       () -> helper.publish(LOOK_AND_STEP_PARAMETERS, StoredPropertySetMessageTools.newMessage(lookAndStepParameters)));
      stopForImpassibilities.set(lookAndStepParameters.getStopForImpassibilities());

      FootstepPlannerParametersBasics footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters("ForLookAndStep");
      footstepPlannerParameterTuner.create(footstepPlannerParameters,
                                           FootstepPlannerParameterKeys.keys,
                                           () -> helper.publish(FootstepPlannerParameters, footstepPlannerParameters.getAllAsStrings()));

      SwingPlannerParametersBasics swingPlannerParameters = helper.getRobotModel().getSwingPlannerParameters("ForLookAndStep");
      swingPlannerParameterTuner.create(swingPlannerParameters, SwingPlannerParameterKeys.keys,
                                        () -> helper.publish(SwingPlannerParameters, swingPlannerParameters.getAllAsStrings()));

      goalAffordance.create(goalPose -> helper.publish(GOAL_INPUT, goalPose), Color.CYAN);
      subGoalGraphic.create(0.027, 0.027 * 6.0, Color.YELLOW);

      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::processImGui3DViewInput);
   }

   public void setGoal(Pose3DBasics goal)
   {
      goalAffordance.setGoalPoseAndPassOn(goal);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      goalAffordance.processImGui3DViewInput(input);
   }

   @Override
   public void update()
   {
      obstacleBoxVisualizer.update();

      if (areGraphicsEnabled())
      {
         footstepPlanGraphic.update();
         commandedFootstepsGraphic.update();
         startAndGoalFootstepsGraphic.update();
         planarRegionsGraphic.update();
         bodyPathPlanGraphic.update();
         if (showReceivedRegions.get())
            receivedRegionsGraphic.update();
      }
   }

   @Override
   public void renderTreeNodeImGuiWidgets()
   {
      ImGui.text("Current state:");
      if (!currentState.isEmpty())
      {
         LookAndStepBehavior.State state = LookAndStepBehavior.State.valueOf(currentState);
         currentStatePlot.render(state.ordinal(), state.name());
      }
      else
      {
         currentStatePlot.render(-1, "");
      }

      if (ImGui.button("Reset"))
      {
         helper.publish(RESET);
      }
      ImGui.sameLine();

      goalAffordance.renderPlaceGoalButton();
      ImGui.text(areGraphicsEnabled() ? "Showing graphics." : "Graphics hidden.");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear")))
         clearGraphics();
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Invert"), invertShowGraphics);
      ImGui.checkbox(labels.get("Received Regions"), showReceivedRegions);

      if (ImGui.checkbox("Operator review", operatorReview))
      {
         helper.publish(OperatorReviewEnabled, operatorReview.get());
      }
      if (ImGui.button("Reject"))
      {
         helper.publish(ReviewApproval, false);
      }
      ImGui.sameLine();
      if (ImGui.button("Approve"))
      {
         helper.publish(ReviewApproval, true);
      }
      footstepPlanningDurationPlot.renderImGuiWidgets();
//      ImGui.text("Footstep planning regions recieved:");
//      steppingRegionsPlot.render(numberOfSteppingRegionsReceived);
      if (ImGui.checkbox(labels.get("Stop for impassibilities"), stopForImpassibilities))
      {
         lookAndStepParameterTuner.changeParameter(LookAndStepBehaviorParameters.stopForImpassibilities, stopForImpassibilities.get());
      }
      impassibilityDetectedPlot.setNextValue(impassibilityDetected.get() ? 1.0f : 0.0f);
      impassibilityDetectedPlot.calculate(impassibilityDetected.get() ? "OBSTRUCTED" : "ALL CLEAR");
//      footholdVolumePlot.render();

//      ImGui.checkbox("Show graphics", showGraphics);
//      ImGui.sameLine();
//      if (ImGui.button("Add support regions once"))
//      {
//         helper.publish(PublishSupportRegions);
//      }

//      if (ImGui.collapsingHeader("Behavior Visualization"))
//      {
//         ImGui.checkbox("Show tuner", showLookAndStepParametersTuner);
//         treePanel.renderWidgetsOnly();
//      }
//      ImGui.pushItemWidth(100.0f);
//      int flags = ImGuiTreeNodeFlags.DefaultOpen;
//      flags += ImGuiTreeNodeFlags.FramePadding;
//      if (ImGui.collapsingHeader("Footstep Planning", flags))
//      {
//      ImGui.separator();

      ImGui.text("Footstep planning:");
      latestFootstepPlannerLogPath.set(latestFootstepPlannerLogPath.get().replace(System.getProperty("user.home"), "~"));
//      ImGui.pushItemWidth(ImGui.getWindowWidth() - 3);
      ImGui.pushItemWidth(340.0f);
      ImGui.text("Latest log:");
      int flags2 = ImGuiInputTextFlags.ReadOnly;
      ImGui.inputText("", latestFootstepPlannerLogPath, flags2);
      ImGui.popItemWidth();
//      ImGui.checkbox("Show tuner", showFootstepPlanningParametersTuner);

      ImGui.text("Rejection reasons:");
      for (int i = 0; i < 5; i++) // Variable number of lines was crashing rendering in imgui-node-editor
      {
         if (latestFootstepPlannerRejectionReasons.size() > i && latestFootstepPlannerRejectionReasons.get(i) != null)
            ImGui.text(latestFootstepPlannerRejectionReasons.get(i).getRight() + "%: "
                       + BipedalFootstepPlannerNodeRejectionReason.values[latestFootstepPlannerRejectionReasons.get(i).getLeft()].name());
         else
            ImGui.text("");
      }
//      if (ImGui.collapsingHeader("Swing Planning"))
//      {
//         ImGui.checkbox("Show tuner", showSwingPlanningParametersTuner);
//      }
   }

   @Override
   public void addChildPanels(ImGuiPanel parentPanel)
   {
      parentPanel.addChild(lookAndStepParameterTuner);
      parentPanel.addChild(footstepPlannerParameterTuner);
      parentPanel.addChild(swingPlannerParameterTuner);
   }

   private boolean areGraphicsEnabled()
   {
      boolean wasTickedRecently = wasTickedRecently(0.5);
      boolean currentStateIsNotEmpty = !currentState.isEmpty();
      boolean isInResetState = currentState.equals(LookAndStepBehavior.State.RESET.name());
      boolean isPlacingGoal = goalAffordance.isPlacingGoal();
      boolean areGraphicsEnabled = (wasTickedRecently && currentStateIsNotEmpty && !isInResetState) || isPlacingGoal;
      if (invertShowGraphics.get())
         areGraphicsEnabled = !areGraphicsEnabled;
      return areGraphicsEnabled;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (areGraphicsEnabled())
      {
         goalAffordance.getRenderables(renderables, pool);
         subGoalGraphic.getRenderables(renderables, pool);
         if (impassibilityDetected.get())
            obstacleBoxVisualizer.getRenderables(renderables, pool);
         footstepPlanGraphic.getRenderables(renderables, pool);
         commandedFootstepsGraphic.getRenderables(renderables, pool);
         startAndGoalFootstepsGraphic.getRenderables(renderables, pool);
         planarRegionsGraphic.getRenderables(renderables, pool);
         bodyPathPlanGraphic.getRenderables(renderables, pool);
         if (showReceivedRegions.get())
            receivedRegionsGraphic.getRenderables(renderables, pool);
      }
   }

   public void clearGraphics()
   {
      goalAffordance.clear();
      subGoalGraphic.clear();
      footstepPlanGraphic.clear();
      commandedFootstepsGraphic.clear();
      startAndGoalFootstepsGraphic.clear();
      planarRegionsGraphic.clear();
      bodyPathPlanGraphic.clear();
      receivedRegionsGraphic.clear();
   }

   @Override
   public void destroy()
   {
      footstepPlanGraphic.destroy();
      commandedFootstepsGraphic.destroy();
      startAndGoalFootstepsGraphic.destroy();
      planarRegionsGraphic.destroy();
      bodyPathPlanGraphic.destroy();
      obstacleBoxVisualizer.dispose();
      receivedRegionsGraphic.destroy();
   }

   public String getWindowName()
   {
      return LookAndStepBehavior.DEFINITION.getName();
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
