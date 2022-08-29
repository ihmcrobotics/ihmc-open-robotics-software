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
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXToolButton;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
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
import us.ihmc.tools.io.WorkspaceDirectory;

import java.util.ArrayList;
import java.util.List;

/**
 *  Possibly extract simple controller controls to a smaller panel class, like remote safety controls or something.
 */
public class GDXTeleoperationManager extends ImGuiPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final WorkspaceDirectory iconDirectory = new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                           "ihmc-high-level-behaviors/src/libgdx/resources/icons");
   private final CommunicationHelper communicationHelper;
   private final ROS2ControllerHelper ros2Helper;
   private final YoVariableClientHelper yoVariableClientHelper;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final ImGuiStoredPropertySetTuner teleoperationParametersTuner = new ImGuiStoredPropertySetTuner("Teleoperation Parameters");
   private final GDXTeleoperationParameters teleoperationParameters;
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
   private final ImString tempImGuiText = new ImString(1000);
   private GDXLiveRobotPartInteractable pelvisInteractable;
   private final GDXWalkPathControlRing walkPathControlRing = new GDXWalkPathControlRing();
   private final boolean interactableExists;

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

      interactableExists = robotSelfCollisionModel != null;
      if (interactableExists)
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
      desiredRobot.create();

      ballAndArrowMidFeetPosePlacement.create(Color.YELLOW);
      ballAndArrowMidFeetPosePlacement.setupIcon(iconDirectory);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ballAndArrowMidFeetPosePlacement::processImGui3DViewInput);
      footstepPlanningParametersTuner.create(footstepPlanning.getFootstepPlannerParameters(),
                                             FootstepPlannerParameterKeys.keys,
                                             footstepPlanning::plan);
      teleoperationParametersTuner.create(teleoperationParameters, GDXTeleoperationParameters.keys);
      teleoperationParametersTuner.registerSlider("Swing time", 0.3f, 2.5f);
      teleoperationParametersTuner.registerSlider("Transfer time", 0.3f, 2.5f);
      teleoperationParametersTuner.registerSlider("Turn aggressiveness", 0.0f, 10.0f);

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

      // Note: hot button for calibrate, open / close hand
      WorkspaceDirectory iconDirectory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources/icons");
      GDXToolButton button;
      // STAND PREP
      Runnable standPrepRunnable = robotLowLevelMessenger::sendStandRequest;
      button = new GDXToolButton("standPrepButton", iconDirectory, "standPrep.png", standPrepRunnable);
      button.setToolTipText("action: Stand prep");
      baseUI.getPrimary3DPanel().addHotButton(button);

      handManager.create(baseUI, iconDirectory, communicationHelper);

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
      pelvisHeightSlider.renderImGuiWidgets();
      chestPitchSlider.renderImGuiWidgets();
      chestYawSlider.renderImGuiWidgets();

      // TODO: sliders for footstep parameters here . . .
      // 2nd
      teleoperationParametersTuner.renderDoublePropertySliders();
      teleoperationParametersTuner.renderADoublePropertyTuner(GDXTeleoperationParameters.trajectoryTime, 0.1, 0.5, 0.0, 30.0, true, "s", "%.2f");

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

      desiredRobot.renderImGuiWidgets();
      ImGui.sameLine();
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

   public List<ImGuiGDXVisualizer> getVisualizers()
   {
      List<ImGuiGDXVisualizer> visualizers = new ArrayList<>();
      visualizers.add(desiredRobot);
      desiredRobot.setActive(true);

      return visualizers;
   }
}
