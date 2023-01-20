package us.ihmc.rdx.ui.teleoperation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
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
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetDoubleWidget;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.affordances.*;
import us.ihmc.rdx.ui.collidables.RDXRobotCollisionModel;
import us.ihmc.rdx.ui.footstepPlanner.RDXFootstepPlanning;
import us.ihmc.rdx.ui.graphics.RDXBodyPathPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.interactable.RDXChestOrientationSlider;
import us.ihmc.rdx.ui.interactable.RDXPelvisHeightSlider;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.rdx.vr.RDXVRContext;
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
import java.util.function.Consumer;

/**
 *  Possibly extract simple controller controls to a smaller panel class, like remote safety controls or something.
 */
public class RDXTeleoperationManager extends ImGuiPanel
{
   RDXBaseUI baseUI;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CommunicationHelper communicationHelper;
   private final ROS2ControllerHelper ros2Helper;
   private final YoVariableClientHelper yoVariableClientHelper;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final RDXTeleoperationParameters teleoperationParameters;
   private final ImGuiStoredPropertySetTuner teleoperationParametersTuner = new ImGuiStoredPropertySetTuner("Teleoperation Parameters");
   private ImGuiStoredPropertySetDoubleWidget swingTimeSlider;
   private ImGuiStoredPropertySetDoubleWidget turnAggressivenessSlider;
   private ImGuiStoredPropertySetDoubleWidget transferTimeSlider;
   private final RDXFootstepPlanGraphic footstepsSentToControllerGraphic;
   private final RDXRobotLowLevelMessenger robotLowLevelMessenger;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final AStarBodyPathPlannerParametersBasics bodyPathPlannerParameters;
   private final ImGuiStoredPropertySetTuner footstepPlanningParametersTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (Teleoperation)");
   private final ImGuiStoredPropertySetTuner bodyPathPlanningParametersTuner = new ImGuiStoredPropertySetTuner("Body Path Planner Parameters (Teleoperation)");
   private final RDXFootstepPlanning footstepPlanning;
   private RDXLegControlMode legControlMode = RDXLegControlMode.DISABLED;
   private final RDXBallAndArrowPosePlacement ballAndArrowMidFeetPosePlacement = new RDXBallAndArrowPosePlacement();
   private final RDXManualFootstepPlacement manualFootstepPlacement = new RDXManualFootstepPlacement();
   private final RDXInteractableFootstepPlan interactableFootstepPlan = new RDXInteractableFootstepPlan();
   private final RDXBodyPathPlanGraphic bodyPathPlanGraphic = new RDXBodyPathPlanGraphic();

   private final RDXPelvisHeightSlider pelvisHeightSlider;
   private final RDXChestOrientationSlider chestPitchSlider;
   private final RDXChestOrientationSlider chestYawSlider;
   private final RDXDesiredRobot desiredRobot;
   private final RDXHandConfigurationManager handManager = new RDXHandConfigurationManager();
   private RDXRobotCollisionModel selfCollisionModel;
   private RDXRobotCollisionModel environmentCollisionModel;
   private RDXArmManager armManager;
   private final ImBoolean showSelfCollisionMeshes = new ImBoolean();
   private final ImBoolean showEnvironmentCollisionMeshes = new ImBoolean();
   private final ImBoolean interactablesEnabled = new ImBoolean(false);

   private final SideDependentList<RDXInteractableFoot> interactableFeet = new SideDependentList<>();
   private final SideDependentList<RDXInteractableHand> interactableHands = new SideDependentList<>();
   private RDXInteractableRobotLink interactablePelvis;
   private final ArrayList<RDXInteractableRobotLink> allInteractableRobotLinks = new ArrayList<>();
   private final SideDependentList<double[]> armHomes = new SideDependentList<>();
   private final SideDependentList<double[]> doorAvoidanceArms = new SideDependentList<>();
   private final ImString tempImGuiText = new ImString(1000);
   private final RDXWalkPathControlRing walkPathControlRing = new RDXWalkPathControlRing();
   private final boolean interactablesAvailable;
   private ImGuiStoredPropertySetDoubleWidget trajectoryTimeSlider;
   private boolean isPlacingFootstep = false;

   public RDXTeleoperationManager(String robotRepoName,
                                  String robotSubsequentPathToResourceFolder,
                                  CommunicationHelper communicationHelper)
   {
      this(robotRepoName, robotSubsequentPathToResourceFolder, communicationHelper, null, null, null);
   }

   public RDXTeleoperationManager(String robotRepoName,
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
      addChild(bodyPathPlanningParametersTuner);
      this.communicationHelper = communicationHelper;
      ROS2NodeInterface ros2Node = communicationHelper.getROS2Node();
      robotModel = communicationHelper.getRobotModel();
      ros2Helper = new ROS2ControllerHelper(ros2Node, robotModel);
      this.yoVariableClientHelper = yoVariableClientHelper;

      this.footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      this.bodyPathPlannerParameters = robotModel.getAStarBodyPathPlannerParameters();

      teleoperationParameters = new RDXTeleoperationParameters(robotRepoName, robotSubsequentPathToResourceFolder, robotModel.getSimpleRobotName());
      teleoperationParameters.load();

      syncedRobot = communicationHelper.newSyncedRobot();

      robotLowLevelMessenger = new RDXRobotLowLevelMessenger(communicationHelper, teleoperationParameters);

      desiredRobot = new RDXDesiredRobot(robotModel, syncedRobot);

      ROS2ControllerHelper slidersROS2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      pelvisHeightSlider = new RDXPelvisHeightSlider(syncedRobot, slidersROS2ControllerHelper, teleoperationParameters);
      // TODO this should update the GDX desired Robot
      chestPitchSlider = new RDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.PITCH, slidersROS2ControllerHelper, teleoperationParameters);
      // TODO this should update the GDX desired robot.
      chestYawSlider = new RDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.YAW, slidersROS2ControllerHelper, teleoperationParameters);

      footstepsSentToControllerGraphic = new RDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      communicationHelper.subscribeToControllerViaCallback(FootstepDataListMessage.class, footsteps ->
      {
         footstepsSentToControllerGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps, "Teleoperation Panel Controller Spy"));
      });
      footstepPlanning = new RDXFootstepPlanning(robotModel, teleoperationParameters, syncedRobot);

      // TODO remove ros from this module, and have it call from the higher level.
      ros2Helper.subscribeViaCallback(ROS2Tools.RAPID_REGIONS, regions ->
      {
         footstepPlanning.setPlanarRegions(regions);
         interactableFootstepPlan.setPlanarRegionsList(regions);
      });
      ros2Helper.subscribeViaCallback(ROS2Tools.HEIGHT_MAP_OUTPUT, heightMap ->
      {
         footstepPlanning.setHeightMapData(heightMap);
         interactableFootstepPlan.setHeightMapMessage(heightMap);
      });


      interactablesAvailable = robotSelfCollisionModel != null;
      if (interactablesAvailable)
      {
         selfCollisionModel = new RDXRobotCollisionModel(robotSelfCollisionModel);
         environmentCollisionModel = new RDXRobotCollisionModel(robotEnvironmentCollisionModel);
         armManager = new RDXArmManager(robotModel,
                                        syncedRobot,
                                        desiredRobot.getDesiredFullRobotModel(),
                                        ros2Helper,
                                        teleoperationParameters);
      }
   }

   public void create(RDXBaseUI baseUI)
   {
      this.baseUI = baseUI;
      desiredRobot.create();

      ballAndArrowMidFeetPosePlacement.create(Color.YELLOW);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ballAndArrowMidFeetPosePlacement::processImGui3DViewInput);
      footstepPlanningParametersTuner.create(footstepPlannerParameters, false, () -> footstepPlanning.setFootstepPlannerParameters(footstepPlannerParameters));
      bodyPathPlanningParametersTuner.create(bodyPathPlannerParameters, false, () -> footstepPlanning.setBodyPathPlannerParameters(bodyPathPlannerParameters));
      teleoperationParametersTuner.create(teleoperationParameters);
      swingTimeSlider = teleoperationParametersTuner.createDoubleSlider(RDXTeleoperationParameters.swingTime, 0.3, 2.5);
      transferTimeSlider = teleoperationParametersTuner.createDoubleSlider(RDXTeleoperationParameters.transferTime, 0.3, 2.5);
      turnAggressivenessSlider = teleoperationParametersTuner.createDoubleSlider(RDXTeleoperationParameters.turnAggressiveness, 0.0, 10.0);

      trajectoryTimeSlider = teleoperationParametersTuner.createDoubleSlider(RDXTeleoperationParameters.trajectoryTime, 0.1, 0.5, "s", "%.2f");

      interactableFootstepPlan.create(baseUI, communicationHelper, syncedRobot, teleoperationParameters, footstepPlanning.getFootstepPlannerParameters());
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableFootstepPlan::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableFootstepPlan::calculate3DViewPick);

      manualFootstepPlacement.create(baseUI, interactableFootstepPlan);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(manualFootstepPlacement::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(manualFootstepPlacement::calculate3DViewPick);

      walkPathControlRing.create(baseUI.getPrimary3DPanel(), robotModel, syncedRobot, teleoperationParameters);

      if (interactablesAvailable)
      {
         selfCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkGreen(), 0.4));
         environmentCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkRed(), 0.4));

         // create the manager for the desired arm setpoints
         armManager.create();

         for (RDXRobotCollidable robotCollidable : environmentCollisionModel.getRobotCollidables())
         {
            RobotDefinition robotDefinition = robotModel.getRobotDefinition();
            FullHumanoidRobotModel fullRobotModel = syncedRobot.getFullRobotModel();
            String modelFileName = RDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(robotCollidable.getRigidBodyName()));

            if (robotCollidable.getRigidBodyName().equals(fullRobotModel.getPelvis().getName()))
            {
               if (interactablePelvis == null)
               {
                  interactablePelvis = new RDXInteractableRobotLink();
                  interactablePelvis.create(robotCollidable,
                                            syncedRobot.getReferenceFrames().getPelvisFrame(),
                                            modelFileName,
                                            baseUI.getPrimary3DPanel());
                  interactablePelvis.setOnSpacePressed(() ->
                  {
                     ros2Helper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(teleoperationParameters.getTrajectoryTime(),
                                                                                                       interactablePelvis.getPose()));
                  });
                  allInteractableRobotLinks.add(interactablePelvis);
               }
               else
               {
                  interactablePelvis.addAdditionalRobotCollidable(robotCollidable);
               }
            }
            for (RobotSide side : RobotSide.values)
            {
               if (RDXInteractableFoot.robotCollidableIsFoot(side, robotCollidable, fullRobotModel))
               {
                  if (!interactableFeet.containsKey(side))
                  {
                     RDXInteractableFoot interactableFoot = new RDXInteractableFoot(side, baseUI, robotCollidable, robotModel, fullRobotModel);
                     interactableFoot.setOnSpacePressed(() ->
                             ros2Helper.publishToController(HumanoidMessageTools.createFootTrajectoryMessage(side,
                                                                                                             teleoperationParameters.getTrajectoryTime(),
                                                                                                             interactableFoot.getPose())));
                     interactableFeet.put(side, interactableFoot);
                     allInteractableRobotLinks.add(interactableFoot);
                  }
                  else
                  {
                     interactableFeet.get(side).addAdditionalRobotCollidable(robotCollidable);
                  }
               }
               if (RDXInteractableHand.robotCollidableIsHand(side, robotCollidable, fullRobotModel))
               {
                  if (!interactableHands.containsKey(side))
                  {
                     RDXInteractableHand interactableHand = new RDXInteractableHand(side, baseUI, robotCollidable, robotModel, syncedRobot, yoVariableClientHelper);
                     interactableHands.put(side, interactableHand);
                     allInteractableRobotLinks.add(interactableHand);
                     // TODO this should probably not handle the space event!
                     // This sends a command to the controller.
                     interactableHand.setOnSpacePressed(armManager.getSubmitDesiredArmSetpointsCallback(side));
                  }
                  else
                  {
                     interactableHands.get(side).addAdditionalRobotCollidable(robotCollidable);
                  }
               }
            }
         }

         baseUI.getVRManager().getContext().addVRPickCalculator(this::calculateVRPick);
         baseUI.getVRManager().getContext().addVRInputProcessor(this::processVRInput);
         baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
         baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
         baseUI.getPrimary3DPanel().addImGuiOverlayAddition(this::renderTooltipsAndContextMenus);
         interactablesEnabled.set(true);
      }

      // STAND PREP
      RDX3DPanelToolbarButton standPrepButton = baseUI.getPrimary3DPanel().addToolbarButton();
      standPrepButton.loadAndSetIcon("icons/standPrep.png");
      standPrepButton.setOnPressed(robotLowLevelMessenger::sendStandRequest);
      standPrepButton.setTooltipText("Stand prep");

      handManager.create(baseUI, communicationHelper);

      baseUI.getPrimaryScene().addRenderableProvider(this::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
   }

   public void update()
   {
      syncedRobot.update();
      desiredRobot.update();
      footstepsSentToControllerGraphic.update();
      boolean isCurrentlyPlacingFootstep = getManualFootstepPlacement().isPlacingFootstep() || ballAndArrowMidFeetPosePlacement.isPlacingGoal();
      if (isPlacingFootstep != isCurrentlyPlacingFootstep)
         baseUI.setModelSceneMouseCollisionEnabled(isCurrentlyPlacingFootstep);
      isPlacingFootstep = isCurrentlyPlacingFootstep;

      if (ballAndArrowMidFeetPosePlacement.getPlacedNotification().poll())
      {
         footstepPlanning.setMidFeetGoalPose(ballAndArrowMidFeetPosePlacement.getGoalPose());
         footstepPlanning.planAsync();
      }

      // TODO: make footsteps from footstepPlan interactable (modifiable)
      if (footstepPlanning.pollHasNewPlanAvailable()) // failed
      {
         FootstepPlannerOutput output = footstepPlanning.pollOutput();
         interactableFootstepPlan.updateFromPlan(output.getFootstepPlan());
         if (output.getBodyPath().size() > 0)
            bodyPathPlanGraphic.generateMeshesAsync(output.getBodyPath());
         else
            bodyPathPlanGraphic.clear();
      }

      if (interactablesEnabled.get())
      {
         walkPathControlRing.update(interactableFootstepPlan);

         if (interactablesAvailable)
         {
            armManager.update(interactableHands);

            selfCollisionModel.update();
            environmentCollisionModel.update();

            for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
               robotPartInteractable.update();
         }
      }

      if (walkPathControlRing.pollIsNewlyModified())
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

      if (legControlMode == RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING)
      {
         interactableFootstepPlan.clear();
         bodyPathPlanGraphic.clear();
      }

      if (legControlMode != RDXLegControlMode.MANUAL_FOOTSTEP_PLACEMENT)
      {
         manualFootstepPlacement.exitPlacement();
      }

      if (legControlMode == RDXLegControlMode.DISABLED)
      {
         interactableFootstepPlan.clear();
         bodyPathPlanGraphic.clear();
      }

      manualFootstepPlacement.update();
      bodyPathPlanGraphic.update();
      interactableFootstepPlan.update();

      if (interactableFootstepPlan.getFootsteps().size() > 0)
      {
         footstepPlanning.setReadyToWalk(false);
         footstepsSentToControllerGraphic.clear();
      }

      boolean allAreDeleted = true;
      if (interactablesAvailable)
      {
         allAreDeleted &= interactablePelvis.isDeleted();
         for (RobotSide side : interactableHands.sides())
         {
            allAreDeleted &= interactableHands.get(side).isDeleted();
         }
         for (RobotSide side : interactableFeet.sides())
         {
            allAreDeleted &= interactableFeet.get(side).isDeleted();
         }
      }
      desiredRobot.setActive(!allAreDeleted);
   }

   private void calculateVRPick(RDXVRContext vrContext)
   {
      if (interactablesAvailable && interactablesEnabled.get())
         environmentCollisionModel.calculateVRPick(vrContext);
   }

   private void processVRInput(RDXVRContext vrContext)
   {
      if (interactablesAvailable)
      {
         for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
            robotPartInteractable.processVRInput(vrContext);

         if (interactablesEnabled.get())
            environmentCollisionModel.processVRInput(vrContext);
      }
   }

   private void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         if (!manualFootstepPlacement.isPlacingFootstep())
            walkPathControlRing.calculate3DViewPick(input);

         if (interactablesAvailable)
         {
            if (input.isWindowHovered())
               environmentCollisionModel.calculate3DViewPick(input);

            for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
               robotPartInteractable.calculate3DViewPick(input);
         }
      }
   }

   // This happens after update.
   private void process3DViewInput(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         if (!manualFootstepPlacement.isPlacingFootstep())
            walkPathControlRing.process3DViewInput(input);

         if (interactablesAvailable)
         {
            environmentCollisionModel.process3DViewInput(input);

            interactablePelvis.process3DViewInput(input);
            for (RobotSide side : interactableFeet.sides())
            {
               if (interactableFeet.get(side).process3DViewInput(input))
               {
                  legControlMode = RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING;
               }
            }
            for (RobotSide side : interactableHands.sides())
            {
               interactableHands.get(side).process3DViewInput(input);
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      robotLowLevelMessenger.renderImGuiWidgets();

      pelvisHeightSlider.renderImGuiWidgets();
      chestPitchSlider.renderImGuiWidgets();
      chestYawSlider.renderImGuiWidgets();

      swingTimeSlider.render();
      transferTimeSlider.render();
      turnAggressivenessSlider.render();
      trajectoryTimeSlider.render();

      ImGui.checkbox(labels.get("Show footstep planner parameter tuner"), footstepPlanningParametersTuner.getIsShowing());
      ImGui.checkbox(labels.get("Show body path planner parameter tuner"), bodyPathPlanningParametersTuner.getIsShowing());
      ImGui.checkbox(labels.get("Show teleoperation parameter tuner"), teleoperationParametersTuner.getIsShowing());

      ImGui.separator();

      if (interactablesAvailable)
      {
         ImGui.checkbox("Interactables enabled", interactablesEnabled);
         ImGui.sameLine();
         if (ImGui.button(labels.get("Delete all")))
         {
            footstepsSentToControllerGraphic.clear();
            ballAndArrowMidFeetPosePlacement.clear();
            manualFootstepPlacement.exitPlacement();
            interactableFootstepPlan.clear();
            bodyPathPlanGraphic.clear();
            walkPathControlRing.delete();
            for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
               robotPartInteractable.delete();
            legControlMode = RDXLegControlMode.DISABLED;
         }
      }

      ImGui.separator();

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

      ImGui.text("Walk path control ring planner:");
      walkPathControlRing.renderImGuiWidgets();

      interactableFootstepPlan.renderImGuiWidgets();
      ImGui.checkbox(labels.get("Show footstep related graphics"), showGraphics);

      ImGui.separator();

      handManager.renderImGuiWidgets();

      if (ImGui.button(labels.get("Set Desired To Current")))
      {
         armManager.setDesiredToCurrent();
         desiredRobot.setDesiredToCurrent();
      }

      if (interactablesAvailable)
      {
         armManager.renderImGuiWidgets();

         ImGui.text("Pelvis:");
         ImGuiTools.previousWidgetTooltip("Send with: Spacebar");
         ImGui.sameLine();
         interactablePelvis.renderImGuiWidgets();


         boolean handInteractablesAreDeleted = true;
         for (RobotSide side : interactableHands.sides())
         {
            ImGui.text(side.getPascalCaseName() + " hand:");
            ImGui.sameLine();
            interactableHands.get(side).renderImGuiWidgets();
            handInteractablesAreDeleted &= interactableHands.get(side).isDeleted();
         }
         desiredRobot.setActive(!handInteractablesAreDeleted);

         if (!handInteractablesAreDeleted)
         {
            // desiredRobot.setPelvisShowing(!pelvisInteractable.isDeleted());
            for (RobotSide side : interactableHands.sides())
               desiredRobot.setArmShowing(side, !interactableHands.get(side).isDeleted()
                                                && armManager.getArmControlMode() == RDXArmControlMode.JOINT_ANGLES);
            // for (RobotSide side : footInteractables.sides())
            //    desiredRobot.setLegShowing(side, !footInteractables.get(side).isDeleted());
         }

         for (RobotSide side : interactableFeet.sides())
         {
            ImGui.text(side.getPascalCaseName() + " foot:");
            ImGui.sameLine();
            if (interactableFeet.get(side).renderImGuiWidgets())
            {
               legControlMode = RDXLegControlMode.SINGLE_SUPPORT_FOOT_POSING;
            }
         }

         ImGui.separator();

         ImGui.text("Show collisions:");
         ImGui.sameLine();
         ImGui.checkbox("Contact", showEnvironmentCollisionMeshes);
         ImGui.sameLine();
         ImGui.checkbox("Avoidance", showSelfCollisionMeshes);
      }

      // TODO: Add transparency sliders
      // TODO: Add motion previews
   }

   private void renderTooltipsAndContextMenus()
   {
      for (RobotSide side : interactableHands.sides())
      {
         RDXInteractableHand interactableHand = interactableHands.get(side);
         if (interactableHand.getContextMenuNotification().poll())
         {
            ImGui.openPopup(labels.get(interactableHand.getContextMenuName()));
         }

         if (ImGui.beginPopup(labels.get(interactableHand.getContextMenuName())))
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
         bodyPathPlanGraphic.getRenderables(renderables, pool);
      }

      if (interactablesEnabled.get())
      {
         if (interactablesAvailable)
         {
            if (showSelfCollisionMeshes.get())
               selfCollisionModel.getRenderables(renderables, pool);
            if (showEnvironmentCollisionMeshes.get())
               environmentCollisionModel.getRenderables(renderables, pool);

            for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
               robotPartInteractable.getVirtualRenderables(renderables, pool);
         }

         walkPathControlRing.getVirtualRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      desiredRobot.destroy();
      walkPathControlRing.destroy();
      footstepsSentToControllerGraphic.destroy();
      bodyPathPlanGraphic.destroy();
      interactableFootstepPlan.destroy();
   }

   public List<RDXVisualizer> getVisualizers()
   {
      List<RDXVisualizer> visualizers = new ArrayList<>();
      visualizers.add(desiredRobot);
      desiredRobot.setActive(true);

      return visualizers;
   }

   public ImBoolean getInteractablesEnabled()
   {
      return interactablesEnabled;
   }

   public RDXRobotCollisionModel getSelfCollisionModel()
   {
      return selfCollisionModel;
   }

   public RDXHandConfigurationManager getHandManager()
   {
      return handManager;
   }

   public RDXTeleoperationParameters getTeleoperationParameters()
   {
      return teleoperationParameters;
   }

   public RDXManualFootstepPlacement getManualFootstepPlacement()
   {
      return manualFootstepPlacement;
   }
}
