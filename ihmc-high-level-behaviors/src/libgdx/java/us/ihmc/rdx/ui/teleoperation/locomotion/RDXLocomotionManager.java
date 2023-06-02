package us.ihmc.rdx.ui.teleoperation.locomotion;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetBooleanWidget;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetDoubleWidget;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.*;
import us.ihmc.rdx.ui.footstepPlanner.RDXFootstepPlanning;
import us.ihmc.rdx.ui.graphics.RDXBodyPathPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXLegControlMode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * This class provides easy access to everything that involves mobility for the robot's legs.
 * Everything with walking and moving the legs are contained in this class.
 * This allows the features to all be grouped together in the UI making the robot easier to operate.
 */
public class RDXLocomotionManager
{
   private RDXBaseUI baseUI;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final CommunicationHelper communicationHelper;
   private final RDXLocomotionParameters locomotionParameters;
   private final ImGuiStoredPropertySetTuner locomotionParametersTuner = new ImGuiStoredPropertySetTuner("Locomotion Parameters");
   private ImGuiStoredPropertySetBooleanWidget areFootstepsAdjustableCheckbox;
   private ImGuiStoredPropertySetBooleanWidget planSwingTrajectoriesCheckbox;
   private ImGuiStoredPropertySetBooleanWidget replanSwingTrajectoriesOnChangeCheckbox;
   private ImGuiStoredPropertySetDoubleWidget swingTimeSlider;
   private ImGuiStoredPropertySetDoubleWidget transferTimeSlider;
   private ImGuiStoredPropertySetDoubleWidget turnAggressivenessSlider;
   private final ImGuiStoredPropertySetTuner swingFootPlanningParametersTuner = new ImGuiStoredPropertySetTuner("Swing Foot Planning Parameters (Teleoperation)");
   private final SwingPlannerParametersBasics swingFootPlannerParameters;

   private final RDXFootstepPlanGraphic footstepsSentToControllerGraphic;
   private final RDXBodyPathPlanGraphic bodyPathPlanGraphic = new RDXBodyPathPlanGraphic();

   private final SideDependentList<RDXInteractableFoot> interactableFeet = new SideDependentList<>();
   private final RDXBallAndArrowPosePlacement ballAndArrowMidFeetPosePlacement = new RDXBallAndArrowPosePlacement();
   private final RDXInteractableFootstepPlan interactableFootstepPlan = new RDXInteractableFootstepPlan();
   private final RDXFootstepPlanning footstepPlanning;
   private final RDXManualFootstepPlacement manualFootstepPlacement = new RDXManualFootstepPlacement();
   private final RDXWalkPathControlRing walkPathControlRing = new RDXWalkPathControlRing();
   private RDXLegControlMode legControlMode = RDXLegControlMode.DISABLED;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private boolean isPlacingFootstep = false;
   private final PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
   private final AbortWalkingMessage abortWalkingMessage = new AbortWalkingMessage();
   private final ControllerStatusTracker controllerStatusTracker;

   public RDXLocomotionManager(DRCRobotModel robotModel,
                               CommunicationHelper communicationHelper,
                               ROS2SyncedRobotModel syncedRobot,
                               ROS2ControllerHelper ros2Helper,
                               ControllerStatusTracker controllerStatusTracker)
   {
      this.robotModel = robotModel;
      this.communicationHelper = communicationHelper;
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;

//      addChild(swingFootPlanningParametersTuner);

      this.swingFootPlannerParameters = robotModel.getSwingPlannerParameters();

      locomotionParameters = new RDXLocomotionParameters(robotModel.getSimpleRobotName());
      locomotionParameters.load();

      footstepPlanning = new RDXFootstepPlanning(robotModel, locomotionParameters, syncedRobot);

      // TODO remove ros from this module, and have it call from the higher level.
      ros2Helper.subscribeViaCallback(PerceptionAPI.SLAM_OUTPUT_RAPID_REGIONS, regions ->
      {
         footstepPlanning.setPlanarRegionsListMessage(regions);
         interactableFootstepPlan.setPlanarRegionsListMessage(regions);
      });
      ros2Helper.subscribeViaCallback(PerceptionAPI.HEIGHT_MAP_OUTPUT, heightMap ->
      {
         footstepPlanning.setHeightMapData(heightMap);
         interactableFootstepPlan.setHeightMapMessage(heightMap);
      });

      footstepsSentToControllerGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      communicationHelper.subscribeToControllerViaCallback(FootstepDataListMessage.class, footsteps ->
            footstepsSentToControllerGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps,
                                                                                                                "Teleoperation Panel Controller Spy")));
   }

   public void create(RDXBaseUI baseUI)
   {
      this.baseUI = baseUI;

      locomotionParametersTuner.create(locomotionParameters);

      swingFootPlanningParametersTuner.create(swingFootPlannerParameters, false, () -> setSwingParameters(swingFootPlannerParameters));

      areFootstepsAdjustableCheckbox = locomotionParametersTuner.createBooleanCheckbox(RDXLocomotionParameters.areFootstepsAdjustable);
      planSwingTrajectoriesCheckbox = locomotionParametersTuner.createBooleanCheckbox(RDXLocomotionParameters.planSwingTrajectories);
      replanSwingTrajectoriesOnChangeCheckbox = locomotionParametersTuner.createBooleanCheckbox(RDXLocomotionParameters.replanSwingTrajectoriesOnChange);
      swingTimeSlider = locomotionParametersTuner.createDoubleSlider(RDXLocomotionParameters.swingTime, 0.3, 1.5);
      transferTimeSlider = locomotionParametersTuner.createDoubleSlider(RDXLocomotionParameters.transferTime, 0.3, 1.5);
      turnAggressivenessSlider = locomotionParametersTuner.createDoubleSlider(RDXLocomotionParameters.turnAggressiveness, 0.0, 10.0);

      ballAndArrowMidFeetPosePlacement.create(Color.YELLOW);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ballAndArrowMidFeetPosePlacement::processImGui3DViewInput);

      interactableFootstepPlan.create(baseUI, communicationHelper, syncedRobot, locomotionParameters, footstepPlanning.getFootstepPlannerParameters());
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableFootstepPlan::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableFootstepPlan::calculate3DViewPick);

      manualFootstepPlacement.create(syncedRobot, baseUI, interactableFootstepPlan, footstepPlanning.getFootstepPlannerParameters());
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(manualFootstepPlacement::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(manualFootstepPlacement::calculate3DViewPick);

      walkPathControlRing.create(baseUI.getPrimary3DPanel(), robotModel, syncedRobot, locomotionParameters);
   }

   public void update()
   {
      controllerStatusTracker.checkControllerIsRunning();

      swingFootPlannerParameters.setMinimumSwingTime(locomotionParameters.getSwingTime());
      setSwingParameters(swingFootPlannerParameters);

      if (ballAndArrowMidFeetPosePlacement.getPlacedNotification().poll())
      {
         footstepPlanning.setMidFeetGoalPose(ballAndArrowMidFeetPosePlacement.getGoalPose());
         footstepPlanning.planAsync();
      }

      if (footstepPlanning.pollHasNewPlanAvailable()) // failed
      {
         FootstepPlannerOutput output = footstepPlanning.pollOutput();
         interactableFootstepPlan.updateFromPlan(output.getFootstepPlan(), output.getSwingTrajectories());
         if (output.getBodyPath().size() > 0)
            bodyPathPlanGraphic.generateMeshesAsync(output.getBodyPath());
         else
            bodyPathPlanGraphic.clear();
      }

      if (walkPathControlRing.getBecomesModifiedNotification().poll())
      {
         legControlMode = RDXLegControlMode.PATH_CONTROL_RING;
         interactableFootstepPlan.clear();
         bodyPathPlanGraphic.clear();
      }

      if (manualFootstepPlacement.pollIsModeNewlyActivated())
      {
         legControlMode = RDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT;
      }

      if (legControlMode != RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING)
      {
         for (RobotSide side : interactableFeet.sides())
         {
            interactableFeet.get(side).delete();
         }
      }

      if (legControlMode != RDXLegControlMode.PATH_CONTROL_RING)
      {
         walkPathControlRing.delete();
      }

      if (legControlMode != RDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT)
      {
         manualFootstepPlacement.exitPlacement();
      }

      if (legControlMode == RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING || legControlMode == RDXLegControlMode.DISABLED)
      {
         interactableFootstepPlan.clear();
         bodyPathPlanGraphic.clear();
      }

      manualFootstepPlacement.update();
      bodyPathPlanGraphic.update();
      interactableFootstepPlan.update();

      if (interactableFootstepPlan.getNumberOfFootsteps() > 0)
      {
         footstepPlanning.setReadyToWalk(false);
         footstepsSentToControllerGraphic.clear();
      }

      footstepsSentToControllerGraphic.update();

      boolean isCurrentlyPlacingFootstep = getManualFootstepPlacement().isPlacingFootstep() || ballAndArrowMidFeetPosePlacement.isPlacingGoal();
      if (isPlacingFootstep != isCurrentlyPlacingFootstep)
         baseUI.setModelSceneMouseCollisionEnabled(isCurrentlyPlacingFootstep);
      isPlacingFootstep = isCurrentlyPlacingFootstep;
   }

   public void renderImGuiWidgets()
   {
      // Used to calculate whether the Walking Options buttons are active or disabled.
      // This ensures that when the spacebar key is pressed it executes the correct action.
      boolean pauseAvailable = controllerStatusTracker.isWalking();
      boolean continueAvailable = !pauseAvailable && controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps() > 0;
      boolean walkAvailable = !continueAvailable && interactableFootstepPlan.getNumberOfFootsteps() > 0;

      areFootstepsAdjustableCheckbox.renderImGuiWidget();
      planSwingTrajectoriesCheckbox.renderImGuiWidget();
      replanSwingTrajectoriesOnChangeCheckbox.renderImGuiWidget();

      ImGui.checkbox(labels.get("Show swing planner parameter tuner"), swingFootPlanningParametersTuner.getIsShowing());


      swingTimeSlider.renderImGuiWidget();
      transferTimeSlider.renderImGuiWidget();
      turnAggressivenessSlider.renderImGuiWidget();

      ImGui.text("Walking Options:");
      ImGui.sameLine();

      if (ImGui.button(labels.get("Abort")))
      {
         sendAbortWalkingMessage();
      }
      ImGui.sameLine();

      ImGui.beginDisabled(!walkAvailable);
      if (ImGui.button(labels.get("Walk")))
      { // TODO: Add checker here. Make it harder to walk or give warning if the checker is failing
         interactableFootstepPlan.walkFromSteps();
      }
      ImGuiTools.previousWidgetTooltip("Keybind: Space");
      ImGui.sameLine();
      ImGui.endDisabled();

      ImGui.beginDisabled(!pauseAvailable);
      if (ImGui.button(labels.get("Pause")))
      {
         setPauseWalkingAndPublish(true);
      }
      ImGuiTools.previousWidgetTooltip("Keybind: Space");
      ImGui.sameLine();
      ImGui.endDisabled();

      ImGui.beginDisabled(!continueAvailable);
      if (ImGui.button(labels.get("Continue")))
      {
         setPauseWalkingAndPublish(false);
      }
      ImGuiTools.previousWidgetTooltip("Keybind: Space");
      ImGui.endDisabled();

      ImGui.text("Leg control mode: " + legControlMode.name());
      if (ImGui.radioButton(labels.get("Disabled"), legControlMode == RDXLegControlMode.DISABLED))
      {
         legControlMode = RDXLegControlMode.DISABLED;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Manual foostep placement"), legControlMode == RDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT))
      {
         legControlMode = RDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT;
      }
      if (ImGui.radioButton(labels.get("Path control ring"), legControlMode == RDXLegControlMode.PATH_CONTROL_RING))
      {
         legControlMode = RDXLegControlMode.PATH_CONTROL_RING;
         walkPathControlRing.becomeModified(true);
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Single support foot posing"), legControlMode == RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING))
      {
         legControlMode = RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING;
      }

      manualFootstepPlacement.renderImGuiWidgets();

      if (ballAndArrowMidFeetPosePlacement.renderPlaceGoalButton())
         legControlMode = RDXLegControlMode.PATH_CONTROL_RING;

      walkPathControlRing.renderImGuiWidgets();
      interactableFootstepPlan.renderImGuiWidgets();

      // Handles all shortcuts for when the spacebar key is pressed
      if (ImGui.isKeyReleased(ImGuiTools.getSpaceKey()))
      {
         if (walkAvailable)
         {
            interactableFootstepPlan.walkFromSteps();
         }
         else if (pauseAvailable)
         {
            setPauseWalkingAndPublish(true);
         }
         else if (continueAvailable)
         {
            setPauseWalkingAndPublish(false);
         }
      }
   }

   public void updateWalkPathControlRing()
   {
      walkPathControlRing.update(interactableFootstepPlan);
   }

   public void calculateWalkPathControlRing3DViewPick(ImGui3DViewInput input)
   {
      if (!manualFootstepPlacement.isPlacingFootstep())
         walkPathControlRing.calculate3DViewPick(input);
   }

   public void processWalkPathControlRing3dViewInput(ImGui3DViewInput input)
   {
      if (!manualFootstepPlacement.isPlacingFootstep())
         walkPathControlRing.process3DViewInput(input);
   }

   public void destroy()
   {
      walkPathControlRing.destroy();
      footstepsSentToControllerGraphic.destroy();
      bodyPathPlanGraphic.destroy();
      interactableFootstepPlan.destroy();
   }

   public void deleteAll()
   {
      footstepsSentToControllerGraphic.clear();
      ballAndArrowMidFeetPosePlacement.clear();
      manualFootstepPlacement.exitPlacement();
      interactableFootstepPlan.clear();
      bodyPathPlanGraphic.clear();
      walkPathControlRing.delete();

      legControlMode = RDXLegControlMode.DISABLED;
   }

   public void setFootstepPlannerParameters(FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      footstepPlanning.setFootstepPlannerParameters(footstepPlannerParameters);
   }

   public void setBodyPathPlannerParameters(AStarBodyPathPlannerParametersBasics bodyPathPlannerParameters)
   {
      footstepPlanning.setBodyPathPlannerParameters(bodyPathPlannerParameters);
   }

   public void setSwingParameters(SwingPlannerParametersBasics swingParameters)
   {
      interactableFootstepPlan.setSwingPlannerParameters(swingParameters);
      footstepPlanning.setSwingFootPlannerParameters(swingParameters);
   }

   public void setLegControlModeToSingleSupportFootPosing()
   {
      legControlMode = RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING;
   }

   public void sendAbortWalkingMessage()
   {
      communicationHelper.publishToController(abortWalkingMessage);
   }

   public void setPauseWalkingAndPublish(boolean pauseWalking)
   {
      pauseWalkingMessage.setPause(pauseWalking);
      communicationHelper.publishToController(pauseWalkingMessage);

      if (pauseWalking)
         LogTools.info("Commanding Pause Walking...");
      else
         LogTools.info("Commanding Continue Walking...");
   }

   public RDXManualFootstepPlacement getManualFootstepPlacement()
   {
      return manualFootstepPlacement;
   }

   public void getWalkPathControlRingVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      walkPathControlRing.getVirtualRenderables(renderables, pool);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepsSentToControllerGraphic.getRenderables(renderables, pool);
      ballAndArrowMidFeetPosePlacement.getRenderables(renderables, pool);
      manualFootstepPlacement.getRenderables(renderables, pool);
      interactableFootstepPlan.getRenderables(renderables, pool);
      bodyPathPlanGraphic.getRenderables(renderables, pool);
   }

   public RDXLocomotionParameters getLocomotionParameters()
   {
      return locomotionParameters;
   }

   public ImGuiStoredPropertySetTuner getLocomotionParametersTuner()
   {
      return locomotionParametersTuner;
   }

   public ImGuiStoredPropertySetTuner getSwingFootPlanningParametersTuner()
   {
      return swingFootPlanningParametersTuner;
   }
}
