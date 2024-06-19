package us.ihmc.rdx.ui.teleoperation.locomotion;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.*;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.subscribers.FilteredNotification;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.InitialStanceSide;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImGuiSliderDouble;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.*;
import us.ihmc.rdx.ui.affordances.*;
import us.ihmc.rdx.ui.footstepPlanner.RDXFootstepPlanning;
import us.ihmc.rdx.ui.graphics.RDXBodyPathPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.teleoperation.RDXLegControlMode;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.Timer;

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
   private final DefaultFootstepPlannerParametersBasics aStarFootstepPlannerParameters;
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParametersToUse = new DefaultFootstepPlannerParameters();
   private final AStarBodyPathPlannerParametersBasics bodyPathPlannerParameters;
   private final SwingPlannerParametersBasics swingFootPlannerParameters;
   private final Notification locomotionParametersChanged = new Notification();
   private final Notification footstepPlanningParametersChanged = new Notification();
   private final Notification usedFootstepParametersChanged = new Notification();
   private final RDXStoredPropertySetTuner locomotionParametersTuner = new RDXStoredPropertySetTuner("Locomotion Parameters");
   private final RDXStoredPropertySetTuner aStartFootstepPlanningParametersTuner
         = new RDXStoredPropertySetTuner("Footstep Planner Parameters (Teleoperation A*)");
   private final RDXStoredPropertySetTuner turnWalkTurnFootstepPlanningParametersTuner
         = new RDXStoredPropertySetTuner("Footstep Planner Parameters (Teleoperation Turn Walk Turn)");
   private final RDXStoredPropertySetTuner bodyPathPlanningParametersTuner = new RDXStoredPropertySetTuner("Body Path Planner Parameters (Teleoperation)");
   private final RDXStoredPropertySetTuner swingFootPlanningParametersTuner = new RDXStoredPropertySetTuner("Swing Foot Planning Parameters (Teleoperation)");
   private ImGuiStoredPropertySetBooleanWidget areFootstepsAdjustableCheckbox;
   private ImGuiStoredPropertySetBooleanWidget assumeFlatGroundCheckbox;
   private ImGuiStoredPropertySetBooleanWidget planSwingTrajectoriesCheckbox;
   private ImGuiStoredPropertySetBooleanWidget replanSwingTrajectoriesOnChangeCheckbox;
   private ImGuiStoredPropertySetDoubleWidget swingTimeSlider;
   private ImGuiStoredPropertySetDoubleWidget transferTimeSlider;
   private ImGuiSliderDouble stepAggressivenessSlider;
   private ImGuiSliderDouble turnAggressivenessSlider;
   private ImGuiStoredPropertySetEnumWidget initialStanceSideRadioButtons;

   private final RDXFootstepPlanGraphic controllerFootstepQueueGraphic;
   private final RDXBodyPathPlanGraphic bodyPathPlanGraphic = new RDXBodyPathPlanGraphic();

   private final SideDependentList<RDXInteractableFoot> interactableFeet = new SideDependentList<>();
   private final RDXBallAndArrowGoalFootstepPlacement ballAndArrowMidFeetPosePlacement = new RDXBallAndArrowGoalFootstepPlacement();
   private final RDXInteractableFootstepPlan interactableFootstepPlan;
   private final RDXFootstepPlanning footstepPlanning;
   private final RDXManualFootstepPlacement manualFootstepPlacement = new RDXManualFootstepPlacement();
   private final RDXWalkPathControlRing walkPathControlRing = new RDXWalkPathControlRing();
   private RDXLegControlMode legControlMode = RDXLegControlMode.DISABLED;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean collapsedHeader = new ImBoolean(true);
   private boolean isPlacingFootstep = false;
   private final PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage();
   private final AbortWalkingMessage abortWalkingMessage = new AbortWalkingMessage();
   private final ControllerStatusTracker controllerStatusTracker;
   private final Notification abortedNotification = new Notification();
   private final FilteredNotification<FootstepQueueStatusMessage> footstepQueueNotification
         = new FilteredNotification<>(new FootstepQueueAcceptanceFunction());
   private final Timer footstepPlanningCompleteTimer = new Timer();

   // Used for UI logic
   private boolean wasPlanning = false;

   public RDXLocomotionManager(DRCRobotModel robotModel,
                               CommunicationHelper communicationHelper,
                               ROS2SyncedRobotModel syncedRobot,
                               ControllerStatusTracker controllerStatusTracker,
                               RDXPanel teleoperationPanel)
   {
      this.robotModel = robotModel;
      this.communicationHelper = communicationHelper;
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;

      locomotionParameters = new RDXLocomotionParameters(robotModel.getSimpleRobotName());
      locomotionParameters.load();
      aStarFootstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      bodyPathPlannerParameters = robotModel.getAStarBodyPathPlannerParameters();
      swingFootPlannerParameters = robotModel.getSwingPlannerParameters();

      teleoperationPanel.addChild(locomotionParametersTuner);
      teleoperationPanel.addChild(aStartFootstepPlanningParametersTuner);
      teleoperationPanel.addChild(turnWalkTurnFootstepPlanningParametersTuner);
      teleoperationPanel.addChild(bodyPathPlanningParametersTuner);
      teleoperationPanel.addChild(swingFootPlanningParametersTuner);

      footstepPlanning = new RDXFootstepPlanning(robotModel,
                                                 syncedRobot,
                                                 controllerStatusTracker,
                                                 locomotionParameters, footstepPlannerParametersToUse,
                                                 bodyPathPlannerParameters,
                                                 swingFootPlannerParameters);
      interactableFootstepPlan = new RDXInteractableFootstepPlan(controllerStatusTracker);
      controllerFootstepQueueGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
   }

   public void create(RDXBaseUI baseUI)
   {
      this.baseUI = baseUI;

      controllerStatusTracker.registerAbortedListener(abortedNotification);
      controllerStatusTracker.getFootstepTracker().registerFootstepQueuedMessageListener(footstepQueueNotification);
      locomotionParameters.addAnyPropertyChangedListener(locomotionParametersChanged);
      aStarFootstepPlannerParameters.addAnyPropertyChangedListener(footstepPlanningParametersChanged);
      footstepPlannerParametersToUse.addAnyPropertyChangedListener(usedFootstepParametersChanged);

      locomotionParametersTuner.create(locomotionParameters);
      aStartFootstepPlanningParametersTuner.create(aStarFootstepPlannerParameters, false);
      bodyPathPlanningParametersTuner.create(bodyPathPlannerParameters, false);
      swingFootPlanningParametersTuner.create(swingFootPlannerParameters, false);

      assumeFlatGroundCheckbox = locomotionParametersTuner.createBooleanCheckbox(RDXLocomotionParameters.assumeFlatGround);
      areFootstepsAdjustableCheckbox = locomotionParametersTuner.createBooleanCheckbox(RDXLocomotionParameters.areFootstepsAdjustable);
      planSwingTrajectoriesCheckbox = locomotionParametersTuner.createBooleanCheckbox(RDXLocomotionParameters.planSwingTrajectories);
      replanSwingTrajectoriesOnChangeCheckbox = locomotionParametersTuner.createBooleanCheckbox(RDXLocomotionParameters.replanSwingTrajectoriesOnChange);
      swingTimeSlider = locomotionParametersTuner.createDoubleSlider(RDXLocomotionParameters.swingTime, 0.3, 1.5);
      transferTimeSlider = locomotionParametersTuner.createDoubleSlider(RDXLocomotionParameters.transferTime, 0.1, 1.5);
      stepAggressivenessSlider = new ImGuiSliderDouble("Step Aggressiveness", "", aStarFootstepPlannerParameters.getIdealFootstepLength() / aStarFootstepPlannerParameters.getMaxStepReach());
      turnAggressivenessSlider = new ImGuiSliderDouble("Turn Aggressiveness", "", 0.5);
      initialStanceSideRadioButtons = locomotionParametersTuner.createEnumRadioButtons(RDXLocomotionParameters.initialStanceSide, InitialStanceSide.values());

      ballAndArrowMidFeetPosePlacement.create(Color.YELLOW, syncedRobot);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ballAndArrowMidFeetPosePlacement::processImGui3DViewInput);

      interactableFootstepPlan.create(baseUI,
                                      communicationHelper,
                                      syncedRobot,
                                      locomotionParameters, footstepPlannerParametersToUse,
                                      swingFootPlannerParameters);
      baseUI.getVRManager().getContext().addVRPickCalculator(interactableFootstepPlan::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(interactableFootstepPlan::processVRInput);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableFootstepPlan::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableFootstepPlan::calculate3DViewPick);

      manualFootstepPlacement.create(syncedRobot, baseUI, interactableFootstepPlan, footstepPlannerParametersToUse);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(manualFootstepPlacement::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(manualFootstepPlacement::calculate3DViewPick);

      walkPathControlRing.create(baseUI.getPrimary3DPanel(), robotModel, syncedRobot, footstepPlannerParametersToUse);

      baseUI.getPrimary3DPanel().addImGuiOverlayAddition(() -> renderOverlayElements(baseUI.getPrimary3DPanel()));
   }

   public void update()
   {
      controllerStatusTracker.checkControllerIsRunning();

      if (footstepQueueNotification.pollFiltered())
      {
         controllerFootstepQueueGraphic.generateMeshesAsync(footstepQueueNotification.read(), "Controller Queue");
      }
      controllerFootstepQueueGraphic.update(); // Will happen once the async mesh generation has completed on a later tick

      if (abortedNotification.poll())
      {
         deleteAll();
      }

      footstepPlannerParametersToUse.set(aStarFootstepPlannerParameters);

      footstepPlannerParametersToUse.setIdealFootstepLength(stepAggressivenessSlider.getDoubleValue() * aStarFootstepPlannerParameters.getMaxStepReach());
      footstepPlannerParametersToUse.setIdealBackStepLength(stepAggressivenessSlider.getDoubleValue() * -aStarFootstepPlannerParameters.getMinStepLength());
      footstepPlannerParametersToUse.setIdealSideStepWidth(stepAggressivenessSlider.getDoubleValue() * aStarFootstepPlannerParameters.getMaxStepWidth());

      if (!locomotionParameters.getPerformAStarSearch())
      {
         footstepPlannerParametersToUse.setMaxStepYaw(turnAggressivenessSlider.getDoubleValue() * aStarFootstepPlannerParameters.getMaxStepYaw());
         footstepPlannerParametersToUse.setMinStepYaw(turnAggressivenessSlider.getDoubleValue() * aStarFootstepPlannerParameters.getMinStepYaw());
      }

      swingFootPlannerParameters.setMinimumSwingTime(locomotionParameters.getSwingTime());

      boolean parametersChanged = locomotionParametersChanged.poll();
      parametersChanged |= footstepPlanningParametersChanged.poll();
      parametersChanged |= usedFootstepParametersChanged.poll();

      if (parametersChanged)
      {
         footstepPlannerParametersToUse.setEnableExpansionMask(locomotionParameters.getAssumeFlatGround());
      }

      if (ballAndArrowMidFeetPosePlacement.getPlacedNotification().poll() || (parametersChanged && ballAndArrowMidFeetPosePlacement.isPlaced()))
      {
         footstepPlanning.queueAsynchronousPlanning(ballAndArrowMidFeetPosePlacement.getGoalPose());
      }

      if (walkPathControlRing.getGoalUpdatedNotification().poll() || (parametersChanged && walkPathControlRing.isSelected()))
      {
         footstepPlanning.queueAsynchronousPlanning(walkPathControlRing.getGoalPose());
      }

      footstepPlanning.update();

      if (footstepPlanning.getPlannerOutputNotification().poll())
      {
         FootstepPlannerOutput output = footstepPlanning.getPlannerOutputNotification().read();
         interactableFootstepPlan.updateFromPlan(output.getFootstepPlan(), output.getSwingTrajectories());
         if (output.getBodyPath().size() > 0)
            bodyPathPlanGraphic.generateMeshesAsync(output.getBodyPath());
         else
            bodyPathPlanGraphic.clear();
      }

      if (walkPathControlRing.getBecomesModifiedNotification().poll())
      {
         legControlMode = RDXLegControlMode.PATH_CONTROL_RING;
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

      boolean isCurrentlyPlacingFootstep =
            getManualFootstepPlacement().isPlacingFootstep() || ballAndArrowMidFeetPosePlacement.isPlacingGoal() || walkPathControlRing.isSelected();
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

      if (ImGui.button(labels.get("Disable Leg Mode")))
      {
         legControlMode = RDXLegControlMode.DISABLED;
      }

      ImGui.sameLine();
      ImGui.text("Leg Mode: " + legControlMode.name());

      swingTimeSlider.renderImGuiWidget();
      transferTimeSlider.renderImGuiWidget();
      stepAggressivenessSlider.render(0.0, 1.5);
      turnAggressivenessSlider.render(0.0, 2.0);

      ImGui.separator();
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
      ImGuiTools.previousWidgetTooltip("Space");
      ImGui.sameLine();
      ImGui.endDisabled();

      ImGui.beginDisabled(!pauseAvailable);
      if (ImGui.button(labels.get("Pause")))
      {
         setPauseWalkingAndPublish(true);
      }
      ImGuiTools.previousWidgetTooltip("Space");
      ImGui.sameLine();
      ImGui.endDisabled();

      ImGui.beginDisabled(!continueAvailable);
      if (ImGui.button(labels.get("Continue")))
      {
         setPauseWalkingAndPublish(false);
      }
      ImGuiTools.previousWidgetTooltip("Space");
      ImGui.endDisabled();

      manualFootstepPlacement.renderImGuiWidgets();

      ImGui.text("Goal Planning");
      ImGui.sameLine();
      if (ballAndArrowMidFeetPosePlacement.renderPlaceGoalButton())
         legControlMode = RDXLegControlMode.PATH_CONTROL_RING;

      ImGui.separator();
      walkPathControlRing.renderImGuiWidgets();

      initialStanceSideRadioButtons.renderImGuiWidget();
      if (ImGui.collapsingHeader(labels.get("Footstep Planning Options"), collapsedHeader))
      {
         ImGui.indent();
         assumeFlatGroundCheckbox.renderImGuiWidget();
         areFootstepsAdjustableCheckbox.renderImGuiWidget();
         planSwingTrajectoriesCheckbox.renderImGuiWidget();
         replanSwingTrajectoriesOnChangeCheckbox.renderImGuiWidget();
         ImGui.unindent();
      }

      // Handles all shortcuts for when the spacebar key is pressed
      if (ImGui.isKeyReleased(ImGuiTools.getSpaceKey()) && !ImGui.getIO().getWantCaptureKeyboard())
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
      walkPathControlRing.update();
   }
   
   public void calculateWalkPathControlRingVRPick(RDXVRContext vrContext)
   {
      if (!manualFootstepPlacement.isPlacingFootstep())
         walkPathControlRing.calculateVRPick(vrContext);
   }

   public void processWalkPathControlRingVRInput(RDXVRContext vrContext)
   {
      if (!manualFootstepPlacement.isPlacingFootstep())
      {
         if (walkPathControlRing.getFootstepPlannerGoalGizmo().getPathControlRingGizmo().getIsGizmoHoveredVR().get(RobotSide.RIGHT))
         {
            vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
            {
               controller.setBButtonText("Delete all");
               if (controller.getBButtonActionData().bChanged() && controller.getBButtonActionData().bState())
               {
                  deleteAll();
               }
               controller.setAButtonText("Walk");
               if (controller.getAButtonActionData().bChanged() && controller.getAButtonActionData().bState())
               {
                  interactableFootstepPlan.walkFromSteps();
               }
            });
         }

         walkPathControlRing.processVRInput(vrContext);
      }
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
      footstepPlanning.destroy();
      controllerFootstepQueueGraphic.destroy();
      bodyPathPlanGraphic.destroy();
      interactableFootstepPlan.destroy();
   }

   public void deleteAll()
   {
      ballAndArrowMidFeetPosePlacement.clear();
      manualFootstepPlacement.exitPlacement();
      interactableFootstepPlan.clear();
      bodyPathPlanGraphic.clear();
      walkPathControlRing.delete();

      legControlMode = RDXLegControlMode.DISABLED;
   }

   public void setLegControlModeToSingleSupportFootPosing()
   {
      legControlMode = RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING;
   }

   public void sendAbortWalkingMessage()
   {
      communicationHelper.publishToController(abortWalkingMessage);
      // The abort walking message can only be process when the controller is in walking state, this forces the abort to go through
      pauseWalkingMessage.setPause(false);
      communicationHelper.publishToController(pauseWalkingMessage);
   }

   public void setPauseWalkingAndPublish(boolean pauseWalking)
   {
      pauseWalkingMessage.setPause(pauseWalking);
      communicationHelper.publishToController(pauseWalkingMessage);

      if (pauseWalking)
         RDXBaseUI.pushNotification("Commanding pause walking...");
      else
         RDXBaseUI.pushNotification("Commanding continue walking...");
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
      controllerFootstepQueueGraphic.getRenderables(renderables, pool);
      ballAndArrowMidFeetPosePlacement.getRenderables(renderables, pool);
      manualFootstepPlacement.getRenderables(renderables, pool);
      interactableFootstepPlan.getRenderables(renderables, pool);
      // Only render when planning with body path
      if (locomotionParameters.getPlanWithBodyPath())
         bodyPathPlanGraphic.getRenderables(renderables, pool);
   }

   public void renderOverlayElements(RDX3DPanel panel3D)
   {
      boolean isPlanning = footstepPlanning.isPlanning();
      boolean hasPlannedRecently = footstepPlanningCompleteTimer.isRunning(2.0);
      if (isPlanning || hasPlannedRecently)
      {
         if (isPlanning)
         {
            if (!wasPlanning)
            {
               wasPlanning = true;
               panel3D.getNotificationManager().pushNotification("Planning footsteps...");
            }
            footstepPlanningCompleteTimer.reset();
         }
         else
         {
            if (wasPlanning)
            {
               wasPlanning = false;
               panel3D.getNotificationManager().pushNotification("Footstep planning completed.");
            }
         }
      }
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      footstepPlanning.setHeightMapData(heightMapData);
      interactableFootstepPlan.setHeightMapMessage(heightMapData);
   }

   public RDXLocomotionParameters getLocomotionParameters()
   {
      return locomotionParameters;
   }

   public RDXInteractableFootstepPlan getInteractableFootstepPlan()
   {
      return interactableFootstepPlan;
   }
}
