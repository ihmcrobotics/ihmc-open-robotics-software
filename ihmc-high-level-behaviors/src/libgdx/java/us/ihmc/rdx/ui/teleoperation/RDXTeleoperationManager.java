package us.ihmc.rdx.ui.teleoperation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
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
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetDoubleWidget;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.affordances.*;
import us.ihmc.rdx.ui.collidables.RDXRobotCollisionModel;
import us.ihmc.rdx.ui.footstepPlanner.RDXFootstepPlanning;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.rdx.ui.interactable.RDXChestOrientationSlider;
import us.ihmc.rdx.ui.interactable.RDXPelvisHeightSlider;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
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
   private final ImGuiStoredPropertySetTuner footstepPlanningParametersTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (Teleoperation)");
   private final RDXFootstepPlanning footstepPlanning;
   private final RDXBallAndArrowPosePlacement ballAndArrowMidFeetPosePlacement = new RDXBallAndArrowPosePlacement();
   private final RDXManualFootstepPlacement manualFootstepPlacement = new RDXManualFootstepPlacement();
   private final RDXInteractableFootstepPlan interactableFootstepPlan = new RDXInteractableFootstepPlan();
   private final RDXPelvisHeightSlider pelvisHeightSlider;
   private final RDXChestOrientationSlider chestPitchSlider;
   private final RDXChestOrientationSlider chestYawSlider;
   private final RDXDesiredRobot desiredRobot;
   private final RDXHandConfigurationManager handManager = new RDXHandConfigurationManager();
   private RDXRobotCollisionModel selfCollisionModel;
   private RDXRobotCollisionModel environmentCollisionModel;
   private RDXWholeBodyDesiredIKManager wholeBodyDesiredIKManager;
   private final ImBoolean showSelfCollisionMeshes = new ImBoolean();
   private final ImBoolean showEnvironmentCollisionMeshes = new ImBoolean();
   private final ImBoolean interactablesEnabled = new ImBoolean(false);

   private final SideDependentList<RDXFootInteractable> footInteractables = new SideDependentList<>();
   private final SideDependentList<RDXHandInteractable> handInteractables = new SideDependentList<>();
   private final SideDependentList<double[]> armHomes = new SideDependentList<>();
   private final SideDependentList<double[]> doorAvoidanceArms = new SideDependentList<>();
   private final ImString tempImGuiText = new ImString(1000);
   private RDXLiveRobotPartInteractable pelvisInteractable;
   private final RDXWalkPathControlRing walkPathControlRing = new RDXWalkPathControlRing();
   private final boolean interactableExists;
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
      this.communicationHelper = communicationHelper;
      ROS2NodeInterface ros2Node = communicationHelper.getROS2Node();
      robotModel = communicationHelper.getRobotModel();
      ros2Helper = new ROS2ControllerHelper(ros2Node, robotModel);
      this.yoVariableClientHelper = yoVariableClientHelper;

      teleoperationParameters = new RDXTeleoperationParameters(robotRepoName, robotSubsequentPathToResourceFolder, robotModel.getSimpleRobotName());
      teleoperationParameters.load();
      teleoperationParameters.save();

      for (RobotSide side : RobotSide.values)
      {
         armHomes.put(side,
                      new double[] {0.5,
                                    side.negateIfRightSide(0.0),
                                    side.negateIfRightSide(-0.5),
                                    -1.0,
                                    side.negateIfRightSide(0.0),
                                    0.000,
                                    side.negateIfLeftSide(0.0)});
      }
      doorAvoidanceArms.put(RobotSide.LEFT, new double[] {-0.121, -0.124, -0.971, -1.713, -0.935, -0.873, 0.277});
      doorAvoidanceArms.put(RobotSide.RIGHT, new double[] {-0.523, -0.328, 0.586, -2.192, 0.828, 1.009, -0.281});

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
      footstepPlanning = new RDXFootstepPlanning(robotModel, syncedRobot);

      interactableExists = robotSelfCollisionModel != null;
      if (interactableExists)
      {
         selfCollisionModel = new RDXRobotCollisionModel(robotSelfCollisionModel);
         environmentCollisionModel = new RDXRobotCollisionModel(robotEnvironmentCollisionModel);
         wholeBodyDesiredIKManager = new RDXWholeBodyDesiredIKManager(robotModel,
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
      footstepPlanningParametersTuner.create(footstepPlanning.getFootstepPlannerParameters(), footstepPlanning::plan);
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

      if (interactableExists)
      {
         selfCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkGreen(), 0.4));
         environmentCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkRed(), 0.4));

         // create the manager for the desired arm setpoints
         wholeBodyDesiredIKManager.create();

         for (RDXRobotCollisionLink collisionLink : environmentCollisionModel.getCollisionLinks())
         {
            RobotDefinition robotDefinition = robotModel.getRobotDefinition();
            FullHumanoidRobotModel fullRobotModel = syncedRobot.getFullRobotModel();
            String modelFileName = RDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(collisionLink.getRigidBodyName()));

            if (collisionLink.getRigidBodyName().equals(fullRobotModel.getPelvis().getName()))
            {
               if (pelvisInteractable == null)
               {
                  pelvisInteractable = new RDXLiveRobotPartInteractable();
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
               if (RDXFootInteractable.collisionLinkIsFoot(side, collisionLink, fullRobotModel))
               {
                  if (!footInteractables.containsKey(side))
                  {
                     RDXFootInteractable footInteractable = new RDXFootInteractable(side, baseUI, collisionLink, robotModel, fullRobotModel);
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
               if (RDXHandInteractable.collisionLinkIsHand(side, collisionLink, fullRobotModel))
               {
                  if (!handInteractables.containsKey(side))
                  {
                     RDXHandInteractable handInteractable = new RDXHandInteractable(side, baseUI, collisionLink, robotModel, syncedRobot, yoVariableClientHelper);
                     handInteractables.put(side, handInteractable);
                     // TODO this should probably not handle the space event!
                     // This sends a command to the controller.
                     handInteractable.setOnSpacePressed(wholeBodyDesiredIKManager.getSubmitDesiredArmSetpointsCallback(side));
                  }
                  else
                  {
                     handInteractables.get(side).addAdditionalCollisionLink(collisionLink);
                  }
               }
            }
         }

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
      boolean isCurrentlyPlacingFootstep = getManualFootstepPlacement().isPlacingFootstep();
      if (isPlacingFootstep != isCurrentlyPlacingFootstep)
         baseUI.setModelSceneMouseCollisionEnabled(isCurrentlyPlacingFootstep);
      isPlacingFootstep = isCurrentlyPlacingFootstep;

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

         if (interactableExists)
         {
            wholeBodyDesiredIKManager.update(handInteractables);

            selfCollisionModel.update();
            environmentCollisionModel.update();

            for (RobotSide side : handInteractables.sides())
            {
               handInteractables.get(side).update();
            }
         }
      }

      manualFootstepPlacement.update();
      interactableFootstepPlan.update();
      if (interactableFootstepPlan.getFootsteps().size() > 0)
      {
         footstepPlanning.setReadyToWalk(false);
         footstepsSentToControllerGraphic.clear();
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         if (!manualFootstepPlacement.isPlacingFootstep())
            walkPathControlRing.calculate3DViewPick(input);

         if (interactableExists)
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
   }

   // This happens after update.
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         if (!manualFootstepPlacement.isPlacingFootstep())
            walkPathControlRing.process3DViewInput(input);

         if (interactableExists)
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
   }

   public void renderImGuiWidgets()
   {
      robotLowLevelMessenger.renderImGuiWidgets();

      ImGui.text("Arms:");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get("Home " + side.getPascalCaseName())))
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        armHomes.get(side));
            ros2Helper.publishToController(armTrajectoryMessage);
         }
      }
      ImGui.text("Door avoidance arms:");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get(side.getPascalCaseName())))
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        doorAvoidanceArms.get(side));
            ros2Helper.publishToController(armTrajectoryMessage);
         }
      }

      pelvisHeightSlider.renderImGuiWidgets();
      chestPitchSlider.renderImGuiWidgets();
      chestYawSlider.renderImGuiWidgets();

      swingTimeSlider.render();
      transferTimeSlider.render();
      turnAggressivenessSlider.render();
      trajectoryTimeSlider.render();

      ImGui.checkbox(labels.get("Show footstep planner parameter tuner"), footstepPlanningParametersTuner.getIsShowing());
      ImGui.checkbox(labels.get("Show teleoperation parameter tuner"), teleoperationParametersTuner.getIsShowing());

      ImGui.separator();

      manualFootstepPlacement.renderImGuiWidgets();

      ballAndArrowMidFeetPosePlacement.renderPlaceGoalButton();

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
      ImGui.checkbox(labels.get("Show footstep related graphics"), showGraphics);

      ImGui.separator();

      handManager.renderImGuiWidgets();

//      desiredRobot.renderImGuiWidgets();
//      ImGui.sameLine();
      if (ImGui.button(labels.get("Set Desired To Current")))
      {
         wholeBodyDesiredIKManager.setDesiredToCurrent();
         desiredRobot.setDesiredToCurrent();
      }

      if (interactableExists)
      {
         wholeBodyDesiredIKManager.renderImGuiWidgets();
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

         ImGui.text("Pelvis:");
         ImGui.sameLine();
         pelvisInteractable.renderImGuiWidgets();

         for (RobotSide side : handInteractables.sides())
         {
            ImGui.text(side.getPascalCaseName() + " hand:");
            ImGui.sameLine();
            handInteractables.get(side).renderImGuiWidgets();
         }
         for (RobotSide side : footInteractables.sides())
         {
            ImGui.text(side.getPascalCaseName() + " foot:");
            ImGui.sameLine();
            footInteractables.get(side).renderImGuiWidgets();
         }

         boolean allAreDeleted = pelvisInteractable.isDeleted();
         for (RobotSide side : handInteractables.sides())
         {
            allAreDeleted &= handInteractables.get(side).isDeleted();
         }
         for (RobotSide side : footInteractables.sides())
         {
            allAreDeleted &= footInteractables.get(side).isDeleted();
         }
         desiredRobot.setActive(!allAreDeleted);

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
      for (RobotSide side : handInteractables.sides())
      {
         RDXHandInteractable handInteractable = handInteractables.get(side);
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
         if (interactableExists)
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
      }
   }

   public void destroy()
   {
      desiredRobot.destroy();
      walkPathControlRing.destroy();
      footstepsSentToControllerGraphic.destroy();
   }

   public List<RDXVisualizer> getVisualizers()
   {
      List<RDXVisualizer> visualizers = new ArrayList<>();
      visualizers.add(desiredRobot);
      desiredRobot.setActive(true);

      return visualizers;
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
