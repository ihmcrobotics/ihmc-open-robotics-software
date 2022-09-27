package us.ihmc.gdx.ui.teleoperation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDX3DPanelToolbarButton;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetDoubleSlider;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.*;
import us.ihmc.gdx.ui.collidables.GDXRobotCollisionModel;
import us.ihmc.gdx.ui.footstepPlanner.GDXFootstepPlanning;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.ui.interactable.GDXChestOrientationSlider;
import us.ihmc.gdx.ui.interactable.GDXPelvisHeightSlider;
import us.ihmc.gdx.ui.missionControl.processes.RestartableJavaProcess;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.gdx.ui.vr.GDXVRInputMode;
import us.ihmc.gdx.ui.vr.GDXWholeBodyIKStreaming;
import us.ihmc.gdx.ui.yo.GDXContinuousStepping;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.YawPitchRollAxis;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.tools.gui.YoAppearanceTools;

import java.util.ArrayList;
import java.util.List;

/**
 *  Possibly extract simple controller controls to a smaller panel class, like remote safety controls or something.
 */
public class GDXTeleoperationManager extends ImGuiPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private GDXImGuiBasedUI baseUI;
   private final CommunicationHelper communicationHelper;
   private final ROS2ControllerHelper ros2Helper;
   private final YoVariableClientHelper yoVariableClientHelper;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final GDXTeleoperationParameters teleoperationParameters;
   private final ImGuiStoredPropertySetTuner teleoperationParametersTuner = new ImGuiStoredPropertySetTuner("Teleoperation Parameters");
   private ImGuiStoredPropertySetDoubleSlider swingTimeSlider;
   private ImGuiStoredPropertySetDoubleSlider turnAggressivenessSlider;
   private ImGuiStoredPropertySetDoubleSlider transferTimeSlider;
   private final GDXFootstepPlanGraphic footstepsSentToControllerGraphic;
   private final GDXRobotLowLevelMessenger robotLowLevelMessenger;
   private final ImGuiStoredPropertySetTuner footstepPlanningParametersTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (Teleoperation)");
   private final GDXFootstepPlanning footstepPlanning;
   private final GDXVRInputMode vrInputMode = GDXVRInputMode.ARM_MODE;
   private GDXLegControlMode legControlMode = GDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT;
   private final GDXBallAndArrowPosePlacement ballAndArrowMidFeetPosePlacement = new GDXBallAndArrowPosePlacement();
   private final GDXManualFootstepPlacement manualFootstepPlacement = new GDXManualFootstepPlacement();
   private final GDXWalkPathControlRing walkPathControlRing = new GDXWalkPathControlRing();
   private final GDXInteractableFootstepPlan interactableFootstepPlan = new GDXInteractableFootstepPlan();
   private final GDXPelvisHeightSlider pelvisHeightSlider;
   private final GDXChestOrientationSlider chestPitchSlider;
   private final GDXChestOrientationSlider chestYawSlider;
   private final GDXDesiredRobot desiredRobot;
   private final GDXHandConfigurationManager handManager = new GDXHandConfigurationManager();
   private GDXRobotCollisionModel selfCollisionModel;
   private GDXRobotCollisionModel environmentCollisionModel;
   private GDXArmManager armManager;
   private GDXWholeBodyIKStreaming wholeBodyIKStreaming;
   private final ImBoolean showSelfCollisionMeshes = new ImBoolean();
   private final ImBoolean showEnvironmentCollisionMeshes = new ImBoolean();
   private final ImBoolean interactablesEnabled = new ImBoolean(false);
   private GDXLiveRobotPartInteractable pelvisInteractable;
   private final SideDependentList<GDXFootInteractable> footInteractables = new SideDependentList<>();
   private final SideDependentList<GDXHandInteractable> handInteractables = new SideDependentList<>();
   private final ImString tempImGuiText = new ImString(1000);
   private final boolean interactablesAvailable;
   private final GDXContinuousStepping continuousStepping;
   private final ImBoolean joystickOn = new ImBoolean(false);

   public GDXTeleoperationManager(String robotRepoName,
                                  String robotSubsequentPathToResourceFolder,
                                  CommunicationHelper communicationHelper)
   {
      this(robotRepoName, robotSubsequentPathToResourceFolder, communicationHelper, null, null, null);
   }

   public GDXTeleoperationManager(String robotRepoName,
                                  String robotSubsequentPathToResourceFolder,
                                  CommunicationHelper communicationHelper,
                                  RobotCollisionModel robotSelfCollisionModel,
                                  RobotCollisionModel robotEnvironmentCollisionModel,
                                  YoVariableClientHelper yoVariableClientHelper)
   {
      super("Teleoperation");

      setRenderMethod(this::renderImGuiWidgets);
      addChild(teleoperationParametersTuner);
      addChild(footstepPlanningParametersTuner);
      this.communicationHelper = communicationHelper;
      ROS2NodeInterface ros2Node = communicationHelper.getROS2Node();
      robotModel = communicationHelper.getRobotModel();
      ros2Helper = new ROS2ControllerHelper(ros2Node, robotModel);
      this.yoVariableClientHelper = yoVariableClientHelper;

      teleoperationParameters = new GDXTeleoperationParameters(robotRepoName, robotSubsequentPathToResourceFolder, robotModel.getSimpleRobotName());
      teleoperationParameters.load();
      teleoperationParameters.save();

      syncedRobot = communicationHelper.newSyncedRobot();

      robotLowLevelMessenger = new GDXRobotLowLevelMessenger(communicationHelper, teleoperationParameters);

      desiredRobot = new GDXDesiredRobot(robotModel, syncedRobot);

      ROS2ControllerHelper slidersROS2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      pelvisHeightSlider = new GDXPelvisHeightSlider(syncedRobot, slidersROS2ControllerHelper, teleoperationParameters);
      // TODO this should update the GDX desired Robot
      chestPitchSlider = new GDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.PITCH, slidersROS2ControllerHelper, teleoperationParameters);
      // TODO this should update the GDX desired robot.
      chestYawSlider = new GDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.YAW, slidersROS2ControllerHelper, teleoperationParameters);

      footstepsSentToControllerGraphic = new GDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      communicationHelper.subscribeToControllerViaCallback(FootstepDataListMessage.class, footsteps ->
      {
         footstepsSentToControllerGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps, "Teleoperation Panel Controller Spy"));
      });
      footstepPlanning = new GDXFootstepPlanning(robotModel, syncedRobot);

      interactablesAvailable = robotSelfCollisionModel != null;
      if (interactablesAvailable)
      {
         selfCollisionModel = new GDXRobotCollisionModel(robotSelfCollisionModel);
         environmentCollisionModel = new GDXRobotCollisionModel(robotEnvironmentCollisionModel);
         armManager = new GDXArmManager(robotModel,
                                        syncedRobot,
                                        desiredRobot.getDesiredFullRobotModel(),
                                        ros2Helper,
                                        teleoperationParameters);
      }

      continuousStepping = new GDXContinuousStepping(robotModel);
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      create(baseUI, null);
   }

   public void create(GDXImGuiBasedUI baseUI, RestartableJavaProcess kinematicsStreamingToolboxProcess)
   {
      this.baseUI = baseUI;
      desiredRobot.create();

//      vrModeManager.create(baseUI,
//                           syncedRobot,
//                           ros2Helper,
//                           kinematicsStreamingToolboxProcess != null,
//                           teleoperationParameters,
//                           this::renderExtraWidgetsOnVRPanel);

      ballAndArrowMidFeetPosePlacement.create(Color.YELLOW);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ballAndArrowMidFeetPosePlacement::processImGui3DViewInput);
      footstepPlanningParametersTuner.create(footstepPlanning.getFootstepPlannerParameters(),
                                             FootstepPlannerParameterKeys.keys,
                                             footstepPlanning::plan);
      teleoperationParametersTuner.create(teleoperationParameters, GDXTeleoperationParameters.keys);
      swingTimeSlider = teleoperationParametersTuner.createDoubleSlider(GDXTeleoperationParameters.swingTime, 0.3, 2.5);
      transferTimeSlider = teleoperationParametersTuner.createDoubleSlider(GDXTeleoperationParameters.transferTime, 0.3, 2.5);
      turnAggressivenessSlider = teleoperationParametersTuner.createDoubleSlider(GDXTeleoperationParameters.turnAggressiveness, 0.0, 10.0);

      interactableFootstepPlan.create(baseUI, communicationHelper, syncedRobot, teleoperationParameters, footstepPlanning.getFootstepPlannerParameters());
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableFootstepPlan::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableFootstepPlan::calculate3DViewPick);

      manualFootstepPlacement.create(baseUI, interactableFootstepPlan);

      walkPathControlRing.create(baseUI,
                                 baseUI.getPrimary3DPanel(),
                                 robotModel,
                                 syncedRobot,
                                 teleoperationParameters,
                                 communicationHelper,
                                 ros2Helper,
                                 footstepPlanning.getFootstepPlannerParameters());

      if (interactablesAvailable)
      {
         selfCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkGreen(), 0.4));
         environmentCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkRed(), 0.4));

         // create the manager for the desired arm setpoints
         armManager.create();

         for (GDXRobotCollisionLink collisionLink : environmentCollisionModel.getCollisionLinks())
         {
            RobotDefinition robotDefinition = robotModel.getRobotDefinition();
            FullHumanoidRobotModel fullRobotModel = syncedRobot.getFullRobotModel();
            String modelFileName = GDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(collisionLink.getRigidBodyName()));

            if (collisionLink.getRigidBodyName().equals(fullRobotModel.getPelvis().getName()))
            {
               if (pelvisInteractable == null)
               {
                  pelvisInteractable = new GDXLiveRobotPartInteractable();
                  pelvisInteractable.create(collisionLink,
                                            syncedRobot.getReferenceFrames().getPelvisFrame(),
                                            modelFileName,
                                            baseUI.getPrimary3DPanel());
                  pelvisInteractable.setOnSpacePressed(() ->
                  {
                     ros2Helper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(teleoperationParameters.getTrajectoryTime(),
                                                                                                       pelvisInteractable.getPose()));
                  });
               }
               else
               {
                  pelvisInteractable.addAdditionalCollisionLink(collisionLink);
               }
            }
            for (RobotSide side : RobotSide.values)
            {
               if (GDXFootInteractable.collisionLinkIsFoot(side, collisionLink, fullRobotModel))
               {
                  if (!footInteractables.containsKey(side))
                  {
                     GDXFootInteractable footInteractable = new GDXFootInteractable(side, baseUI, collisionLink, robotModel, fullRobotModel);
                     footInteractable.setOnSpacePressed(() ->
                             ros2Helper.publishToController(HumanoidMessageTools.createFootTrajectoryMessage(side,
                                                                                                             teleoperationParameters.getTrajectoryTime(),
                                                                                                             footInteractable.getPose())));
                     footInteractables.put(side, footInteractable);
                  }
                  else
                  {
                     footInteractables.get(side).addAdditionalCollisionLink(collisionLink);
                  }
               }
               if (GDXHandInteractable.collisionLinkIsHand(side, collisionLink, fullRobotModel))
               {
                  if (!handInteractables.containsKey(side))
                  {
                     GDXHandInteractable handInteractable = new GDXHandInteractable(side, baseUI, collisionLink, robotModel, syncedRobot, yoVariableClientHelper);
                     handInteractables.put(side, handInteractable);
                     // TODO this should probably not handle the space event!
                     // This sends a command to the controller.
                     handInteractable.setOnSpacePressed(armManager.getSubmitDesiredArmSetpointsCallback(side));
                  }
                  else
                  {
                     handInteractables.get(side).addAdditionalCollisionLink(collisionLink);
                  }
               }
            }
         }

         if (kinematicsStreamingToolboxProcess != null)
         {
            wholeBodyIKStreaming = new GDXWholeBodyIKStreaming(syncedRobot.getRobotModel(), ros2Helper, kinematicsStreamingToolboxProcess);
            wholeBodyIKStreaming.create(baseUI.getVRManager().getContext(), handInteractables);
         }

         baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
         baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
         baseUI.getVRManager().getContext().addVRPickCalculator(this::calculateVRPick);
         baseUI.getVRManager().getContext().addVRInputProcessor(this::processVRInput);
         baseUI.getPrimary3DPanel().addImGuiOverlayAddition(this::renderTooltipsAndContextMenus);
         interactablesEnabled.set(true);
      }

      // STAND PREP
      GDX3DPanelToolbarButton standPrepButton = baseUI.getPrimary3DPanel().addToolbarButton();
      standPrepButton.loadAndSetIcon("icons/standPrep.png");
      standPrepButton.setOnPressed(robotLowLevelMessenger::sendStandRequest);
      standPrepButton.setTooltipText("Stand prep");

      handManager.create(baseUI, communicationHelper);

      baseUI.getPrimaryScene().addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);

      continuousStepping.create(baseUI,ros2Helper, robotModel, teleoperationParameters, syncedRobot, communicationHelper, walkPathControlRing);
   }

   public void update()
   {
      update(false, false);
   }

   public void update(boolean nativesLoaded, boolean nativesNewlyLoaded)
   {
      syncedRobot.update();
      desiredRobot.update();
      footstepsSentToControllerGraphic.update();

      if (ballAndArrowMidFeetPosePlacement.getPlacedNotification().poll())
      {
         footstepPlanning.getMidFeetGoalPose().set(ballAndArrowMidFeetPosePlacement.getGoalPose());
         footstepPlanning.setGoalFootPosesFromMidFeetPose();
         footstepPlanning.setStanceSideToClosestToGoal();
         // TODO: Call planAsync
         footstepPlanning.plan();

         // TODO: make footsteps from footstepPlan interactable (modifiable)
         if (footstepPlanning.isReadyToWalk()) // failed
         {
            interactableFootstepPlan.updateFromPlan(footstepPlanning.getOutput().getFootstepPlan());
         }
      }

      if (interactablesEnabled.get())
      {
         walkPathControlRing.update(interactableFootstepPlan);

         if (interactablesAvailable)
         {
            armManager.update(handInteractables);

            selfCollisionModel.update();
            environmentCollisionModel.update();

            for (RobotSide side : handInteractables.sides())
            {
               handInteractables.get(side).update();
            }
         }
      }

      if (walkPathControlRing.checkIsNwlyModified())
      {
         legControlMode = GDXLegControlMode.PATH_CONTROL_RING;
      }

      if (legControlMode != GDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING)
      {
         for (RobotSide side : footInteractables.sides())
         {
            footInteractables.get(side).delete();
         }
      }

      if (legControlMode != GDXLegControlMode.PATH_CONTROL_RING)
      {
         walkPathControlRing.delete();
      }

      if (legControlMode == GDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING)
      {
         interactableFootstepPlan.clear();
      }

      if (legControlMode != GDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT)
      {
         manualFootstepPlacement.exitPlacement();
      }

      manualFootstepPlacement.update();
      interactableFootstepPlan.update();
      if (interactableFootstepPlan.getFootsteps().size() > 0)
      {
         footstepPlanning.setReadyToWalk(false);
         footstepsSentToControllerGraphic.clear();
      }

//      vrModeManager.update(nativesLoaded, nativesNewlyLoaded);
      if (wholeBodyIKStreaming != null)
      {
         boolean isIKStreamingMode = armManager.getArmControlMode() == GDXArmControlMode.STREAMING;
         wholeBodyIKStreaming.update(isIKStreamingMode);
         if (isIKStreamingMode && armManager.getArmControlModeChanged() && !wholeBodyIKStreaming.getKinematicsStreamingToolboxProcess().isStarted())
         {
            wholeBodyIKStreaming.getKinematicsStreamingToolboxProcess().start();
         }
      }
      continuousStepping.update(true);
   }

   private void calculateVRPick(GDXVRContext vrContext)
   {
      if (interactablesEnabled.get())
      {
         if (interactablesAvailable)
         {
            environmentCollisionModel.calculateVRPick(vrContext);
         }
      }
   }

   private void processVRInput(GDXVRContext vrContext)
   {
      pelvisInteractable.processVRInput(vrContext);
      for (RobotSide side : footInteractables.sides())
      {
         footInteractables.get(side).processVRInput(vrContext);
      }
      for (RobotSide side : handInteractables.sides())
      {
         handInteractables.get(side).processVRInput(vrContext);
      }

      if (vrInputMode == GDXVRInputMode.ARM_MODE)
      {
      }

      if (interactablesEnabled.get())
      {
         if (interactablesAvailable)
         {
            environmentCollisionModel.processVRInput(vrContext);
         }
      }
   }

   private void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         walkPathControlRing.calculate3DViewPick(input);

         if (interactablesAvailable)
         {
            if (input.isWindowHovered())
               environmentCollisionModel.calculate3DViewPick(input);

            pelvisInteractable.calculate3DViewPick(input);
            for (RobotSide side : footInteractables.sides())
            {
               footInteractables.get(side).calculate3DViewPick(input);
            }
            for (RobotSide side : handInteractables.sides())
            {
               handInteractables.get(side).calculate3DViewPick(input);
            }
         }
      }

//      continuousStepping.calculate3DViewPick(input);

   }

   // This happens after update.
   private void process3DViewInput(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         walkPathControlRing.process3DViewInput(input);

         if (interactablesAvailable)
         {
            environmentCollisionModel.process3DViewInput(input);

            pelvisInteractable.process3DViewInput(input);
            for (RobotSide side : footInteractables.sides())
            {
               footInteractables.get(side).process3DViewInput(input);
            }
            for (RobotSide side : handInteractables.sides())
            {
               handInteractables.get(side).process3DViewInput(input);
            }
         }
      }
      boolean ctrlHeld = imgui.internal.ImGui.getIO().getKeyCtrl();
      boolean isPPressed = input.isWindowHovered() && ImGui.isKeyDown('P');
      if (ctrlHeld)
      {
         if (isPPressed)

         {
            teleportCameraToRobotPelvis();
         }
      }
//      continuousStepping.processInput(input);
   }

   private void renderExtraWidgetsOnVRPanel()
   {
      renderSharedImGuiWidgets();
   }

   public void renderImGuiWidgets()
   {
      robotLowLevelMessenger.renderImGuiWidgets();

      if (interactablesAvailable)
      {
         ImGui.checkbox("Interactables enabled", interactablesEnabled);
         ImGui.sameLine();
         if (ImGui.button(labels.get("Delete all")))
         {
            walkPathControlRing.delete();
            pelvisInteractable.delete();
            for (RobotSide side : footInteractables.sides())
               footInteractables.get(side).delete();
            for (RobotSide side : handInteractables.sides())
               handInteractables.get(side).delete();
         }
      }
      teleoperationParametersTuner.renderADoublePropertyTuner(GDXTeleoperationParameters.trajectoryTime, 0.1, 0.5, 0.0, 30.0, true, "s", "%.2f");
      ImGui.checkbox(labels.get("Show teleoperation parameter tuner"), teleoperationParametersTuner.getIsShowing());

      ImGui.separator();
      pelvisHeightSlider.renderImGuiWidgets();
      if (interactablesAvailable)
      {
         ImGui.text("Pelvis pose:");
         ImGuiTools.previousWidgetTooltip("Send with: Spacebar");
         ImGui.sameLine();
         pelvisInteractable.renderImGuiWidgets();
      }
      ImGui.separator();
      chestPitchSlider.renderImGuiWidgets();
      chestYawSlider.renderImGuiWidgets();
      ImGui.separator();

      ImGui.text("Leg control mode: " + legControlMode.name());
      if (ImGui.radioButton(labels.get("Manual foostep placement"), legControlMode == GDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT))
      {
         legControlMode = GDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Path control ring"), legControlMode == GDXLegControlMode.PATH_CONTROL_RING))
      {
         legControlMode = GDXLegControlMode.PATH_CONTROL_RING;
      }
      if (ImGui.radioButton(labels.get("Joystick walking"), legControlMode == GDXLegControlMode.JOYSTICK_WALKING))
      {
         legControlMode = GDXLegControlMode.JOYSTICK_WALKING;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Single support foot posing"), legControlMode == GDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING))
      {
         legControlMode = GDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING;
      }

      if (manualFootstepPlacement.renderImGuiWidgets())
         legControlMode = GDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT;

      if (ballAndArrowMidFeetPosePlacement.renderPlaceGoalButton())
         legControlMode = GDXLegControlMode.PATH_CONTROL_RING;

      ImGui.text("Walk path control ring planner:");
      walkPathControlRing.renderImGuiWidgets();

      interactableFootstepPlan.renderImGuiWidgets();

      ImGui.sameLine();
      if (ImGui.button(labels.get("Delete All")))
      {
         footstepsSentToControllerGraphic.clear();
         ballAndArrowMidFeetPosePlacement.clear();
         manualFootstepPlacement.exitPlacement();
         interactableFootstepPlan.clear();
         walkPathControlRing.delete();
      }

      if (interactablesAvailable)
      {
         for (RobotSide side : footInteractables.sides())
         {
            ImGui.text(side.getPascalCaseName() + " foot:");
            ImGui.sameLine();
            if (footInteractables.get(side).renderImGuiWidgets())
            {
               legControlMode = GDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING;
            }
         }
      }

      ImGui.checkbox(labels.get("Show footstep related graphics"), showGraphics);
      swingTimeSlider.render();
      transferTimeSlider.render();
      turnAggressivenessSlider.render();
      ImGui.checkbox(labels.get("Show footstep planner parameter tuner"), footstepPlanningParametersTuner.getIsShowing());

      ImGui.separator();

      handManager.renderImGuiWidgets();

//      desiredRobot.renderImGuiWidgets();
//      ImGui.sameLine();
//      if (ImGui.button(labels.get("Set Desired To Current")))
//      {
//         wholeBodyDesiredIKManager.setDesiredToCurrent();
//         desiredRobot.setDesiredToCurrent();
//      }

      if (interactablesAvailable)
      {
         armManager.renderImGuiWidgets();

         boolean handInteractablesAreDeleted = true;
         for (RobotSide side : handInteractables.sides())
         {
            ImGui.text(side.getPascalCaseName() + " hand:");
            ImGui.sameLine();
            handInteractables.get(side).renderImGuiWidgets();
            handInteractablesAreDeleted &= handInteractables.get(side).isDeleted();
         }

         desiredRobot.setActive(!handInteractablesAreDeleted);

         if (!handInteractablesAreDeleted)
         {
//            desiredRobot.setPelvisShowing(!pelvisInteractable.isDeleted());
            for (RobotSide side : handInteractables.sides())
               desiredRobot.setArmShowing(side, !handInteractables.get(side).isDeleted()
                                                && armManager.getArmControlMode() == GDXArmControlMode.JOINT_ANGLES);
//            for (RobotSide side : footInteractables.sides())
//               desiredRobot.setLegShowing(side, !footInteractables.get(side).isDeleted());
         }

         ImGui.separator();

         ImGui.text("Show collisions:");
         ImGui.sameLine();
         ImGui.checkbox("Contact", showEnvironmentCollisionMeshes);
         ImGui.sameLine();
         ImGui.checkbox("Avoidance", showSelfCollisionMeshes);
      }

      renderSharedImGuiWidgets();

      // TODO: Add transparency sliders
      // TODO: Add motion previews
   }

   private void renderSharedImGuiWidgets()
   {
      if (armManager.getArmControlMode() == GDXArmControlMode.STREAMING)
      {
         wholeBodyIKStreaming.renderImGuiWidgets();
      }
   }

   private void renderTooltipsAndContextMenus()
   {
      for (RobotSide side : handInteractables.sides())
      {
         GDXHandInteractable handInteractable = handInteractables.get(side);
         if (handInteractable.getContextMenuNotification().poll())
         {
            ImGui.openPopup(labels.get(handInteractable.getContextMenuName()));
         }

         if (ImGui.beginPopup(labels.get(handInteractable.getContextMenuName())))
         {
            ImGui.text("Real robot joint angles:");

            tempImGuiText.clear();

            tempImGuiText.set(buildJointAnglesString(side, syncedRobot.getFullRobotModel()));
            ImGui.inputTextMultiline(labels.getHidden(side.getPascalCaseName() + "RealRobotJointAngles"), tempImGuiText, 0, 60, ImGuiInputTextFlags.ReadOnly);

            ImGui.text("Desired joint angles:");
            tempImGuiText.set(buildJointAnglesString(side, desiredRobot.getDesiredFullRobotModel()));
            ImGui.inputTextMultiline(labels.getHidden(side.getPascalCaseName() + "DesiredRobotJointAngles"), tempImGuiText, 0, 60, ImGuiInputTextFlags.ReadOnly);

            if (ImGui.menuItem("Close"))
               ImGui.closeCurrentPopup();
            ImGui.endPopup();
         }
      }
   }

   private String buildJointAnglesString(RobotSide side, FullHumanoidRobotModel fullRobotModel)
   {
      StringBuilder jointAnglesString = new StringBuilder();

      ArmJointName[] armJointNames = robotModel.getJointMap().getArmJointNames();
      int i = 0;
      for (ArmJointName armJoint : armJointNames)
      {
         double q = fullRobotModel.getArmJoint(side, armJoint).getQ();
         jointAnglesString.append(FormattingTools.getFormattedDecimal3D(q));

         if (i < armJointNames.length - 1)
         {
            jointAnglesString.append(",");
         }
         if ((i - 2) % 3 == 0)
         {
            jointAnglesString.append("\n");
         }
         else
         {
            jointAnglesString.append(" ");
         }

         ++i;
      }
      return jointAnglesString.toString();
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      desiredRobot.getRenderables(renderables, pool);

      if (showGraphics.get())
      {
         footstepsSentToControllerGraphic.getRenderables(renderables, pool);
         ballAndArrowMidFeetPosePlacement.getRenderables(renderables, pool);
         manualFootstepPlacement.getRenderables(renderables, pool);
         interactableFootstepPlan.getRenderables(renderables, pool);
      }

      if (interactablesEnabled.get())
      {
         if (interactablesAvailable)
         {
            if (showSelfCollisionMeshes.get())
            {
               selfCollisionModel.getRenderables(renderables, pool);
            }
            if (showEnvironmentCollisionMeshes.get())
            {
               environmentCollisionModel.getRenderables(renderables, pool);
            }

            pelvisInteractable.getVirtualRenderables(renderables, pool);
            for (RobotSide side : footInteractables.sides())
            {
               footInteractables.get(side).getVirtualRenderables(renderables, pool);
            }
            for (RobotSide side : handInteractables.sides())
            {
               handInteractables.get(side).getVirtualRenderables(renderables, pool);
            }
         }

         walkPathControlRing.getVirtualRenderables(renderables, pool);

         if (armManager.getArmControlMode() == GDXArmControlMode.STREAMING)
         {
            wholeBodyIKStreaming.getVirtualRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      desiredRobot.destroy();
      walkPathControlRing.destroy();
      footstepsSentToControllerGraphic.destroy();
//      vrModeManager.destroy();
      if (wholeBodyIKStreaming != null)
         wholeBodyIKStreaming.destroy();
   }

   public List<ImGuiGDXVisualizer> getVisualizers()
   {
      List<ImGuiGDXVisualizer> visualizers = new ArrayList<>();
      visualizers.add(desiredRobot);
      desiredRobot.setActive(true);

      return visualizers;
   }

   public GDXRobotCollisionModel getSelfCollisionModel()
   {
      return selfCollisionModel;
   }

   public void teleportCameraToRobotPelvis()
   {
      RigidBodyTransform robotTransform = syncedRobot.getReferenceFrames().getPelvisFrame().getTransformToWorldFrame();
      GDXFocusBasedCamera camera = baseUI.getPrimary3DPanel().getCamera3D();
      camera.setFocusPointPose(robotTransform);
   }

   public GDXHandConfigurationManager getHandManager()
   {
      return handManager;
   }

//   public GDXVRModeManager getVRModeManager()
//   {
//      return vrModeManager;
//   }
}
