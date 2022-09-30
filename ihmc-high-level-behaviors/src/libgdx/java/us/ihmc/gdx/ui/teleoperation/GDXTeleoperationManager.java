package us.ihmc.gdx.ui.teleoperation;

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
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
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
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
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
   GDXImGuiBasedUI baseUI;
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
   private final GDXBallAndArrowPosePlacement ballAndArrowMidFeetPosePlacement = new GDXBallAndArrowPosePlacement();
   private final GDXManualFootstepPlacement manualFootstepPlacement = new GDXManualFootstepPlacement();
   private final GDXInteractableFootstepPlan interactableFootstepPlan = new GDXInteractableFootstepPlan();
   private final GDXPelvisHeightSlider pelvisHeightSlider;
   private final GDXChestOrientationSlider chestPitchSlider;
   private final GDXChestOrientationSlider chestYawSlider;
   private final GDXDesiredRobot desiredRobot;
   private final GDXHandConfigurationManager handManager = new GDXHandConfigurationManager();
   private GDXRobotCollisionModel selfCollisionModel;
   private GDXRobotCollisionModel environmentCollisionModel;
   private GDXWholeBodyDesiredIKManager wholeBodyDesiredIKManager;
   private final ImBoolean showSelfCollisionMeshes = new ImBoolean();
   private final ImBoolean showEnvironmentCollisionMeshes = new ImBoolean();
   private final ImBoolean interactablesEnabled = new ImBoolean(false);

   private final SideDependentList<GDXFootInteractable> footInteractables = new SideDependentList<>();
   private final SideDependentList<GDXHandInteractable> handInteractables = new SideDependentList<>();
   private final SideDependentList<double[]> armHomes = new SideDependentList<>();
   private final SideDependentList<double[]> doorAvoidanceArms = new SideDependentList<>();
   private final ImString tempImGuiText = new ImString(1000);
   private GDXLiveRobotPartInteractable pelvisInteractable;
   private final GDXWalkPathControlRing walkPathControlRing = new GDXWalkPathControlRing();
   private final boolean interactablesAvailable;

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
         wholeBodyDesiredIKManager = new GDXWholeBodyDesiredIKManager(robotModel,
                                                                      syncedRobot,
                                                                      desiredRobot.getDesiredFullRobotModel(),
                                                                      ros2Helper,
                                                                      teleoperationParameters);
      }
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      this.baseUI = baseUI;
      desiredRobot.create();

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
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(manualFootstepPlacement::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(manualFootstepPlacement::calculate3DViewPick);

      walkPathControlRing.create(baseUI.getPrimary3DPanel(), robotModel, syncedRobot, teleoperationParameters);

      if (interactablesAvailable)
      {
         selfCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkGreen(), 0.4));
         environmentCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkRed(), 0.4));

         // create the manager for the desired arm setpoints
         wholeBodyDesiredIKManager.create();

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
      GDX3DPanelToolbarButton standPrepButton = baseUI.getPrimary3DPanel().addToolbarButton();
      standPrepButton.loadAndSetIcon("icons/standPrep.png");
      standPrepButton.setOnPressed(robotLowLevelMessenger::sendStandRequest);
      standPrepButton.setTooltipText("Stand prep");

      handManager.create(baseUI, communicationHelper);

      baseUI.getPrimaryScene().addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
   }

   public void update()
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
   }

   // This happens after update.
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         if (!manualFootstepPlacement.isPlacingFootstep())
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

      if (interactablesAvailable)
      {
         for (RobotSide side : footInteractables.sides())
         {
            ImGui.text(side.getPascalCaseName() + " foot:");
            ImGui.sameLine();
            footInteractables.get(side).renderImGuiWidgets();
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

      if (interactablesAvailable)
      {
         wholeBodyDesiredIKManager.renderImGuiWidgets();
         boolean handInteractablesAreDeleted = true;
         for (RobotSide side : handInteractables.sides())
         {
            ImGui.text(side.getPascalCaseName() + " hand:");
            ImGui.sameLine();
            handInteractables.get(side).renderImGuiWidgets();
            handInteractablesAreDeleted &= handInteractables.get(side).isDeleted();
         }

         desiredRobot.setActive(!handInteractablesAreDeleted);

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
      }
   }

   public void destroy()
   {
      desiredRobot.destroy();
      walkPathControlRing.destroy();
      footstepsSentToControllerGraphic.destroy();
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

   public GDXHandConfigurationManager getHandManager()
   {
      return handManager;
   }

   public GDXTeleoperationParameters getTeleoperationParameters()
   {
      return teleoperationParameters;
   }
}
