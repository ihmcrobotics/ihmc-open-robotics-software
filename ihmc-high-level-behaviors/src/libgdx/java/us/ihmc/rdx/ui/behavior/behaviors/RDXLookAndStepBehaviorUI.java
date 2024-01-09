package us.ihmc.rdx.ui.behavior.behaviors;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiInputTextFlags;
import imgui.internal.ImGui;
import com.badlogic.gdx.graphics.*;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImString;
import perception_msgs.msg.dds.HeightMapMessage;
import std_msgs.msg.dds.Bool;
import toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonMessage;
import toolbox_msgs.msg.dds.FootstepPlannerRejectionReasonsMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.ReferenceBasedIdealStepCalculator;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXBallAndArrowPosePlacement;
import us.ihmc.rdx.ui.behavior.registry.RDXBehaviorUIInterface;
import us.ihmc.rdx.ui.graphics.RDXBodyPathPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXBoxVisualizer;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.yo.ImGuiYoDoublePlot;
import us.ihmc.rdx.ui.yo.ImPlotYoHelperDoublePlotLine;
import us.ihmc.rdx.ui.graphics.RDXHeightMapGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.rdx.visualizers.RDXSphereAndArrowGraphic;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Set;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.*;

public class RDXLookAndStepBehaviorUI extends RDXBehaviorUIInterface
{
   private final BehaviorHelper helper;
   private String currentState = "";
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean operatorReview = new ImBoolean(true);
   private final ImGuiEnumPlot currentStatePlot = new ImGuiEnumPlot(1000, 250, 15);
   private long numberOfSteppingRegionsReceived = 0;
   private final ImGuiPlot steppingRegionsPlot = new ImGuiPlot("", 1000, 250, 15);
   private final ImGuiMovingPlot impassibilityDetectedPlot = new ImGuiMovingPlot("Impassibility", 1000, 250, 15);
   private final IHMCROS2Input<Bool> impassibilityDetected;
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
   private FootstepPlannerRejectionReasonsMessage latestFootstepPlannerRejectionReasons = new FootstepPlannerRejectionReasonsMessage();

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

   private ImDouble ballAndArrowX = new ImDouble(7.0);
   private ImDouble ballAndArrowY = new ImDouble(0.5);
   private ImDouble ballAndArrowYaw = new ImDouble(0.1);
   private ImDoubleWrapper referenceAlpha;

   public RDXLookAndStepBehaviorUI(BehaviorHelper helper)
   {
      this.helper = helper;
      helper.subscribeViaCallback(CURRENT_STATE, state -> currentState = state.getDataAsString());
      helper.subscribeViaCallback(OPERATOR_REVIEW_ENABLED_STATUS, message -> operatorReview.set(message.getData()));
      helper.subscribeViaCallback(PLANAR_REGIONS_FOR_UI, regions ->
      {
         ++numberOfSteppingRegionsReceived;
         if (regions != null)
            planarRegionsGraphic.generateMeshesAsync(PlanarRegionMessageConverter.convertToPlanarRegionsList(regions));
      });
      heightMapGraphic.getRenderGroundPlane().set(false);
      helper.subscribeViaCallback(HEIGHT_MAP_FOR_UI, heightMapGraphic::generateMeshesAsync);
      helper.subscribeViaCallback(RECEIVED_PLANAR_REGIONS_FOR_UI, regions ->
      {
         if (regions != null)
            receivedRegionsGraphic.generateMeshesAsync(PlanarRegionMessageConverter.convertToPlanarRegionsList(regions));
      });
      helper.subscribeViaCallback(GOAL_STATUS, goalAffordance::setGoalPoseNoCallbacks);
      helper.subscribeViaCallback(SUB_GOAL_FOR_UI, subGoalGraphic::setToPose);
      helper.subscribeViaCallback(BODY_PATH_PLAN_FOR_UI, bodyPath ->
      {
         if (bodyPath != null)
            Gdx.app.postRunnable(() -> bodyPathPlanGraphic.generateMeshes(bodyPath.getPoses()));
      });
      footstepPlanGraphic = new RDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      commandedFootstepsGraphic = new RDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      startAndGoalFootstepsGraphic = new RDXFootstepPlanGraphic(helper.getRobotModel().getContactPointParameters().getControllerFootGroundContactPoints());
      footstepPlanGraphic.setOpacity(0.2);
      helper.subscribeViaCallback(PLANNED_FOOTSTEPS_FOR_UI, footsteps ->
      {
         reviewingBodyPath = false;
         numberOfPlannedSteps = footsteps.getMinimalFootsteps().size();
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.convertMinimalFootstepListMessage(footsteps));
      });
      helper.subscribeViaCallback(LAST_COMMANDED_FOOTSTEPS, footsteps ->
      {
         commandedFootstepsGraphic.generateMeshesAsync(MinimalFootstep.convertMinimalFootstepListMessage(footsteps));
      });
      startAndGoalFootstepsGraphic.setColor(RobotSide.LEFT, Color.BLUE);
      startAndGoalFootstepsGraphic.setColor(RobotSide.RIGHT, Color.BLUE);
      startAndGoalFootstepsGraphic.setOpacity(0.4);
      helper.subscribeViaCallback(IMMINENT_FOOT_POSES_FOR_UI, message ->
            startAndGoalFootstepsGraphic.generateMeshesAsync(MinimalFootstep.convertMinimalFootstepListMessage(message)));
      footstepPlanningDurationPlot = new ImPlotYoHelperDoublePlotLine("LookAndStepBehavior.footstepPlanningDuration", 10.0, helper);
      helper.subscribeViaCallback(FOOTSTEP_PLANNER_LATEST_LOG_PATH, message -> latestFootstepPlannerLogPath.set(message.getDataAsString()));
      helper.subscribeViaCallback(FOOTSTEP_PLANNER_REJECTION_REASONS, reasons ->
      {
         latestFootstepPlannerRejectionReasons = reasons;
         numberOfPlannedSteps = 0;
         planningFailedNotification.set();
      });
      footholdVolumePlot = new ImGuiYoDoublePlot("footholdVolume", helper, 1000, 250, 15);
      impassibilityDetected = helper.subscribe(IMPASSIBILITY_DETECTED);
      obstacleBoxVisualizer.setColor(Color.RED);
      helper.subscribeViaCallback(OBSTACLE, box3DMessage ->
      {
         Box3D box3D = new Box3D();
         box3D.set(box3DMessage.getPose(), box3DMessage.getSize());
         obstacleBoxVisualizer.generateMeshAsync(box3D);
      });
      helper.subscribeViaCallback(RESET_FOR_UI, goalAffordance::clear);
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
      referenceAlpha = new ImDoubleWrapper(footstepPlannerRemotePropertySet.getStoredPropertySet(),
                                           FootstepPlannerParameterKeys.referencePlanAlpha,
                                           alpha ->
                                           {
                                              if (ImGuiTools.volatileInputDouble("Reference alpha", alpha))
                                              {
                                                 footstepPlannerRemotePropertySet.setPropertyChanged();
                                              }
                                           });
   }

   @Override
   public void create(RDXBaseUI baseUI)
   {
      goalAffordance.create(goalPose -> helper.publish(GOAL_COMMAND, goalPose), Color.CYAN);
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
         helper.publish(LookAndStepBehaviorAPI.OPERATOR_REVIEW_ENABLED_COMMAND, operatorReview.get());
      }
      if (ImGui.button("Reject"))
      {
         helper.publish(REVIEW_APPROVAL, false);
      }
      ImGui.sameLine();
      if (ImGui.button("Approve"))
      {
         helper.publish(REVIEW_APPROVAL, true);
      }
      footstepPlanningDurationPlot.renderImGuiWidgets();
      ImGui.text("Footstep planning regions recieved:");
      steppingRegionsPlot.render(numberOfSteppingRegionsReceived);
      stopForImpassibilities.renderImGuiWidget();
      impassibilityDetectedPlot.setNextValue(impassibilityDetected.getLatest().getData() ? 1.0f : 0.0f);
      impassibilityDetectedPlot.calculate(impassibilityDetected.getLatest().getData() ? "OBSTRUCTED" : "ALL CLEAR");

      referenceAlpha.renderImGuiWidget();

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
      ImGui.inputText(labels.getHidden("Planner log path"), latestFootstepPlannerLogPath, flags2);
      ImGui.popItemWidth();
//      ImGui.checkbox("Show tuner", showFootstepPlanningParametersTuner);

      ImGui.text("Rejection reasons:");
      for (int i = 0; i < 5; i++) // Variable number of lines was crashing rendering in imgui-node-editor
      {
         if (latestFootstepPlannerRejectionReasons.getRejectionReasons().size() > i)
         {
            FootstepPlannerRejectionReasonMessage rejectionReasonMessage = latestFootstepPlannerRejectionReasons.getRejectionReasons().get(i);
            ImGui.text(rejectionReasonMessage.getRejectionPercentage() + "%: "
                       + BipedalFootstepPlannerNodeRejectionReason.values[(int) rejectionReasonMessage.getReason()].name());
         }
         else
         {
            ImGui.text("");
         }
      }
//      if (ImGui.collapsingHeader("Swing Planning"))
//      {
//         ImGui.checkbox("Show tuner", showSwingPlanningParametersTuner);
//      }
   }

   @Override
   public void addChildPanels(RDXPanel parentPanel)
   {
      parentPanel.addChild(lookAndStepRemotePropertySet.createPanel());
      parentPanel.addChild(footstepPlannerRemotePropertySet.createPanel());
      parentPanel.addChild(swingPlannerRemotePropertySet.createPanel());
   }

   private boolean areGraphicsEnabled()
   {
      boolean wasTickedRecently = getState().getIsActive();
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
            if (impassibilityDetected.getLatest().getData())
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
      return "Look and Step";
   }

   public String getName()
   {
      return "Look and Step";
   }
}
