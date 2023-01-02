package us.ihmc.rdx.ui.behavior.behaviors;

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
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXBallAndArrowPosePlacement;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIDefinition;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.rdx.ui.graphics.RDXBodyPathPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXBoxVisualizer;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.yo.ImGuiYoDoublePlot;
import us.ihmc.rdx.ui.yo.ImPlotYoHelperDoublePlotLine;
import us.ihmc.rdx.visualizers.RDXHeightMapGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.rdx.visualizers.RDXSphereAndArrowGraphic;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class RDXLookAndStepBehaviorUI extends RDXBehaviorUIInterface
{
   public static final RDXBehaviorUIDefinition DEFINITION = new RDXBehaviorUIDefinition(LookAndStepBehavior.DEFINITION,
                                                                                        RDXLookAndStepBehaviorUI::new);

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
   private ImBooleanWrapper stopForImpassibilities;
   private final ImPlotYoHelperDoublePlotLine footstepPlanningDurationPlot;
   private final ImGuiYoDoublePlot footholdVolumePlot;
   private final ImBoolean invertShowGraphics = new ImBoolean(false);
   private final ImBoolean showReceivedRegions = new ImBoolean(false);
   private final ImBoolean showHeightMap = new ImBoolean(true);
   private final ImBoolean showPlannedSteps = new ImBoolean(true);
   private final ImBoolean showLastCommandedSteps = new ImBoolean(false);

   private boolean reviewingBodyPath = true;
   private final ImString latestFootstepPlannerLogPath = new ImString();
   private ArrayList<Pair<Integer, Double>> latestFootstepPlannerRejectionReasons = new ArrayList<>();

   private final RDXSphereAndArrowGraphic subGoalGraphic = new RDXSphereAndArrowGraphic();
   private final RDXPlanarRegionsGraphic planarRegionsGraphic = new RDXPlanarRegionsGraphic();
   private final RDXPlanarRegionsGraphic receivedRegionsGraphic = new RDXPlanarRegionsGraphic();
   private final RDXHeightMapGraphic heightMapGraphic = new RDXHeightMapGraphic();
   private final RDXBodyPathPlanGraphic bodyPathPlanGraphic = new RDXBodyPathPlanGraphic();
   private final RDXFootstepPlanGraphic footstepPlanGraphic;
   private final RDXFootstepPlanGraphic commandedFootstepsGraphic;
   private final RDXFootstepPlanGraphic startAndGoalFootstepsGraphic;
   private final ImGuiRemoteROS2StoredPropertySet lookAndStepRemotePropertySet;
   private final ImGuiRemoteROS2StoredPropertySet footstepPlannerRemotePropertySet;
   private final ImGuiRemoteROS2StoredPropertySet swingPlannerRemotePropertySet;
   private final RDXBallAndArrowPosePlacement goalAffordance = new RDXBallAndArrowPosePlacement();
   private final RDXBoxVisualizer obstacleBoxVisualizer = new RDXBoxVisualizer();
   private final Notification planningFailedNotification = new Notification();
   private volatile int numberOfPlannedSteps = 0;

   public RDXLookAndStepBehaviorUI(BehaviorHelper helper)
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
      heightMapGraphic.getRenderGroundPlane().set(false);
      helper.subscribeViaCallback(HEIGHT_MAP_FOR_UI, heightMapGraphic::generateMeshesAsync);
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
      footstepPlanGraphic = new RDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      commandedFootstepsGraphic = new RDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      startAndGoalFootstepsGraphic = new RDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      footstepPlanGraphic.setTransparency(0.2);
      latestPlannedFootsteps = helper.subscribeViaReference(PlannedFootstepsForUI, new ArrayList<>());
      latestCommandedFootsteps = helper.subscribeViaReference(LastCommandedFootsteps, new ArrayList<>());
      helper.subscribeViaCallback(PlannedFootstepsForUI, footsteps ->
      {
         reviewingBodyPath = false;
         numberOfPlannedSteps = footsteps.size();
         footstepPlanGraphic.generateMeshesAsync(footsteps);
      });
      helper.subscribeViaCallback(LastCommandedFootsteps, commandedFootstepsGraphic::generateMeshesAsync);
      startAndGoalFootstepsGraphic.setColor(RobotSide.LEFT, Color.BLUE);
      startAndGoalFootstepsGraphic.setColor(RobotSide.RIGHT, Color.BLUE);
      startAndGoalFootstepsGraphic.setTransparency(0.4);
      helper.subscribeViaCallback(ImminentFootPosesForUI, startAndGoalFootstepsGraphic::generateMeshesAsync);
      footstepPlanningDurationPlot = new ImPlotYoHelperDoublePlotLine("LookAndStepBehavior.footstepPlanningDuration", 10.0, helper);
      helper.subscribeViaCallback(FootstepPlannerLatestLogPath, latestFootstepPlannerLogPath::set);
      helper.subscribeViaCallback(FootstepPlannerRejectionReasons, reasons ->
      {
         latestFootstepPlannerRejectionReasons = reasons;
         numberOfPlannedSteps = 0;
         planningFailedNotification.set();
      });
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
      lookAndStepRemotePropertySet = new ImGuiRemoteROS2StoredPropertySet(helper, helper.getRobotModel().getLookAndStepParameters(), PARAMETERS);
      footstepPlannerRemotePropertySet = new ImGuiRemoteROS2StoredPropertySet(helper,
                                                                              helper.getRobotModel().getFootstepPlannerParameters("ForLookAndStep"),
                                                                              FOOTSTEP_PLANNING_PARAMETERS);
      swingPlannerRemotePropertySet = new ImGuiRemoteROS2StoredPropertySet(helper,
                                                                           helper.getRobotModel().getSwingPlannerParameters("ForLookAndStep"),
                                                                           SWING_PLANNER_PARAMETERS);
      stopForImpassibilities = new ImBooleanWrapper(lookAndStepRemotePropertySet.getStoredPropertySet(),
                                                    LookAndStepBehaviorParameters.stopForImpassibilities,
                                                    stopForImpassibilities ->
                                                    {
                                                       if (ImGui.checkbox(labels.get("Stop for impassibilities"), stopForImpassibilities))
                                                       {
                                                          lookAndStepRemotePropertySet.setPropertyChanged();
                                                       }
                                                    });
   }

   @Override
   public void create(RDXBaseUI baseUI)
   {

      goalAffordance.create(goalPose -> helper.publish(GOAL_INPUT, goalPose), Color.CYAN);
      goalAffordance.setOnStartPositionPlacement(() -> baseUI.setModelSceneMouseCollisionEnabled(true));
      goalAffordance.setOnEndPositionPlacement(() -> baseUI.setModelSceneMouseCollisionEnabled(false));
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
      if (planningFailedNotification.poll())
      {
         footstepPlanGraphic.clear();
      }

      if (areGraphicsEnabled())
      {
         footstepPlanGraphic.update();
         commandedFootstepsGraphic.update();
         startAndGoalFootstepsGraphic.update();
         planarRegionsGraphic.update();
         heightMapGraphic.update();
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
      ImGui.checkbox(labels.get("Show height map"), showHeightMap);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear")))
         clearGraphics();
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Invert"), invertShowGraphics);
      ImGui.checkbox(labels.get("Received Regions"), showReceivedRegions);
      ImGui.sameLine();
      ImGui.text("Steps:");
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Planned"), showPlannedSteps);
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Commanded"), showLastCommandedSteps);

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
      stopForImpassibilities.renderImGuiWidget();
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

      ImGui.text("Footstep planning: Planned steps: " + numberOfPlannedSteps);
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
      parentPanel.addChild(lookAndStepRemotePropertySet.createPanel());
      parentPanel.addChild(footstepPlannerRemotePropertySet.createPanel());
      parentPanel.addChild(swingPlannerRemotePropertySet.createPanel());
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
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (areGraphicsEnabled())
      {
         if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         {
            goalAffordance.getRenderables(renderables, pool);
            subGoalGraphic.getRenderables(renderables, pool);
            if (showPlannedSteps.get())
               footstepPlanGraphic.getRenderables(renderables, pool);
            if (showLastCommandedSteps.get())
               commandedFootstepsGraphic.getRenderables(renderables, pool);
            startAndGoalFootstepsGraphic.getRenderables(renderables, pool);
            bodyPathPlanGraphic.getRenderables(renderables, pool);
         }
         if (sceneLevels.contains(RDXSceneLevel.MODEL))
         {
            if (impassibilityDetected.get())
               obstacleBoxVisualizer.getRenderables(renderables, pool);
            planarRegionsGraphic.getRenderables(renderables, pool);
            if (showHeightMap.get())
               heightMapGraphic.getRenderables(renderables, pool);
            if (showReceivedRegions.get())
               receivedRegionsGraphic.getRenderables(renderables, pool);
         }
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
      heightMapGraphic.generateMeshesAsync(new HeightMapMessage());
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
      heightMapGraphic.destroy();
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
