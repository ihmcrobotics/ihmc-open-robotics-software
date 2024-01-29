package us.ihmc.rdx.ui.teleoperation;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiInputTextFlags;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetDoubleWidget;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.*;
import us.ihmc.rdx.ui.collidables.RDXRobotCollisionModel;
import us.ihmc.rdx.ui.interactable.RDXChestOrientationSlider;
import us.ihmc.rdx.ui.interactable.RDXPelvisHeightSlider;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionManager;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.YawPitchRollAxis;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.tools.gui.YoAppearanceTools;

import java.util.ArrayList;
import java.util.Set;

/**
 * The teleoperation manager is the top level class for managing UI for
 * teleoperation of a humanoid robot. It should contain a bunch of additional
 * "sub managers" and UI tools with clear sub-domains.
 * <br/>
 * This class manages the communications with the robot which include ROS 2
 * and YoVariable Client-Server protocols. It should strive to allow field
 * members access to these communications in order not to duplicate
 * network traffic or overhead. This is not always possible or easy due
 * to threading constraints.
 * <br/>
 * The interactable robot parts are all in this class so they can be shared
 * by the sub managers.
 * <br/>
 * Sub managers:
 * <ul>
 * <li>{@link RDXArmManager Arm manager}</li>
 * <li>{@link RDXHandConfigurationManager Hand configuration manager} - lives inside the arm manager</li>
 * <li>{@link RDXLocomotionManager Locomotion manager}</li>
 * </ul>
 *
 * TODO:
 * <ul>
 * <li>Possibly extract simple controller controls to a smaller panel class, like remote safety controls or something.</li>
 * </ul>
 */
public class RDXTeleoperationManager extends RDXPanel
{
   RDXBaseUI baseUI;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2ControllerHelper ros2Helper;
   private final YoVariableClientHelper yoVariableClientHelper;
   private final DRCRobotModel robotModel;
   private final boolean robotHasArms;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final RDXTeleoperationParameters teleoperationParameters;
   private final RDXStoredPropertySetTuner teleoperationParametersTuner = new RDXStoredPropertySetTuner("Teleoperation Parameters");
   private final RDXRobotLowLevelMessenger robotLowLevelMessenger;

   private final RDXPelvisHeightSlider pelvisHeightSlider;
   private final RDXChestOrientationSlider chestPitchSlider;
   private final RDXChestOrientationSlider chestYawSlider;
   private final RDXDesiredRobot desiredRobot;
   private RDXRobotCollisionModel avoidanceCollisionModel;
   private RDXRobotCollisionModel contactCollisionModel;
   private final ImBoolean showAvoidanceCollisionMeshes = new ImBoolean();
   private final ImBoolean showContactCollisionMeshes = new ImBoolean();
   private RDXArmManager armManager;
   private final RDXLocomotionManager locomotionManager;
   private final ImBoolean interactablesEnabled = new ImBoolean(false);

   private final SideDependentList<RDXInteractableFoot> interactableFeet = new SideDependentList<>();
   private final SideDependentList<RDXInteractableHand> interactableHands = new SideDependentList<>();
   private RDXInteractableRobotLink interactableChest;
   private RDXInteractableRobotLink interactablePelvis;
   private final ArrayList<RDXInteractableRobotLink> allInteractableRobotLinks = new ArrayList<>();
   private final ImString tempImGuiText = new ImString(1000);
   private final ImBoolean interactableSelections = new ImBoolean(true);
   private final boolean interactablesAvailable;
   private ImGuiStoredPropertySetDoubleWidget trajectoryTimeSlider;
   private RDXHumanoidDemoPoses demoPoses;

   /** This tracker should be shared with the sub-managers to keep the state consistent. */
   private final ControllerStatusTracker controllerStatusTracker;
   private final LogToolsLogger logToolsLogger = new LogToolsLogger();

   /**
    * For use without interactables available. May crash if a YoVariableClient is needed.
    */
   public RDXTeleoperationManager(CommunicationHelper communicationHelper)
   {
      this(communicationHelper, null, null, null);
   }

   /**
    * Enable interactables and use a YoVariable client to show wrist force arrows on
    * some robots.
    */
   public RDXTeleoperationManager(CommunicationHelper communicationHelper,
                                  RobotCollisionModel robotSelfCollisionModel,
                                  RobotCollisionModel robotSelectionCollisionModel,
                                  YoVariableClientHelper yoVariableClientHelper)
   {
      super("Teleoperation");

      setRenderMethod(this::renderImGuiWidgets);
      addChild(teleoperationParametersTuner);
      robotModel = communicationHelper.getRobotModel();
      robotHasArms = robotModel.getRobotVersion().hasBothArms();
      ros2Helper = communicationHelper.getControllerHelper();
      this.yoVariableClientHelper = yoVariableClientHelper;

      teleoperationParameters = new RDXTeleoperationParameters(robotModel.getSimpleRobotName());
      teleoperationParameters.load();

      syncedRobot = communicationHelper.newSyncedRobot();

      robotLowLevelMessenger = new RDXRobotLowLevelMessenger(communicationHelper, teleoperationParameters);

      desiredRobot = new RDXDesiredRobot(robotModel, syncedRobot);
      desiredRobot.setSceneLevels(RDXSceneLevel.VIRTUAL);

      pelvisHeightSlider = new RDXPelvisHeightSlider(syncedRobot, ros2Helper, teleoperationParameters);
      // TODO this should update the GDX desired Robot
      chestPitchSlider = new RDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.PITCH, ros2Helper, teleoperationParameters);
      // TODO this should update the GDX desired robot.
      chestYawSlider = new RDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.YAW, ros2Helper, teleoperationParameters);

      controllerStatusTracker = new ControllerStatusTracker(logToolsLogger, ros2Helper.getROS2NodeInterface(), robotModel.getSimpleRobotName());

      locomotionManager = new RDXLocomotionManager(robotModel, communicationHelper, syncedRobot, ros2Helper, controllerStatusTracker, this);

      interactablesAvailable = robotSelfCollisionModel != null;
      if (interactablesAvailable)
      {
         avoidanceCollisionModel = new RDXRobotCollisionModel(robotSelfCollisionModel);
         contactCollisionModel = new RDXRobotCollisionModel(robotSelectionCollisionModel);
      }

      if (robotHasArms)
      {
         // create the manager for the desired arm setpoints
         armManager = new RDXArmManager(communicationHelper,
                                        robotModel,
                                        syncedRobot,
                                        desiredRobot,
                                        teleoperationParameters,
                                        interactableHands);
      }

      RDXBaseUI.getInstance().getKeyBindings().register("Delete all Interactables", "Shift + Escape");
   }

   public void create(RDXBaseUI baseUI)
   {
      this.baseUI = baseUI;
      desiredRobot.create();

      // This button is created before locomotion manager to make the toolbar button ordering correct
      RDX3DPanelToolbarButton deleteAllInteractablesButton = baseUI.getPrimary3DPanel().addToolbarButton();
      deleteAllInteractablesButton.loadAndSetIcon("icons/deleteAll.png");
      deleteAllInteractablesButton.setOnPressed(this::clearInteractablesAndLocomotionGraphics);
      deleteAllInteractablesButton.setTooltipText("Delete All Interactables (Shift + Escape)");

      locomotionManager.create(baseUI);

      teleoperationParametersTuner.create(teleoperationParameters);

      trajectoryTimeSlider = teleoperationParametersTuner.createDoubleInput(RDXTeleoperationParameters.trajectoryTime, 0.1, 0.5, "s", "%.2f");

      if (interactablesAvailable)
      {
         avoidanceCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkGreen(), 0.4));
         contactCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkRed(), 0.4));

         for (RDXRobotCollidable robotCollidable : contactCollisionModel.getRobotCollidables())
         {
            RobotDefinition robotDefinition = robotModel.getRobotDefinition();
            FullHumanoidRobotModel fullRobotModel = syncedRobot.getFullRobotModel();
            String modelFileName = RDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(robotCollidable.getRigidBodyName()));

            if (robotCollidable.getRigidBodyName().equals(fullRobotModel.getChest().getName()))
            {
               if (interactableChest == null)
               {
                  interactableChest = new RDXInteractableRobotLink();
                  interactableChest.create(robotCollidable,
                                            syncedRobot.getReferenceFrames().getChestFrame(),
                                            modelFileName,
                                            baseUI.getPrimary3DPanel());
                  interactableChest.setActionExecutor(() ->
                  {
                     ros2Helper.publishToController(HumanoidMessageTools.createChestTrajectoryMessage(teleoperationParameters.getTrajectoryTime(),
                                                                                                     interactableChest.getPose().getOrientation()));
                  });
                  allInteractableRobotLinks.add(interactableChest);
               }
               else
               {
                  interactableChest.addAdditionalRobotCollidable(robotCollidable);
               }
            }
            if (robotCollidable.getRigidBodyName().equals(fullRobotModel.getPelvis().getName()))
            {
               if (interactablePelvis == null)
               {
                  interactablePelvis = new RDXInteractableRobotLink();
                  interactablePelvis.create(robotCollidable,
                                            syncedRobot.getReferenceFrames().getPelvisFrame(),
                                            modelFileName,
                                            baseUI.getPrimary3DPanel());
                  interactablePelvis.setActionExecutor(() ->
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
                     interactableFoot.setActionExecutor(() ->
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
               if (robotHasArms && RDXInteractableHand.robotCollidableIsHand(side, robotCollidable, fullRobotModel))
               {
                  if (!interactableHands.containsKey(side))
                  {
                     RDXInteractableHand interactableHand = new RDXInteractableHand(side, baseUI, robotCollidable, robotModel, syncedRobot, yoVariableClientHelper);
                     interactableHands.put(side, interactableHand);
                     allInteractableRobotLinks.add(interactableHand);
                  }
                  else
                  {
                     interactableHands.get(side).addAdditionalRobotCollidable(robotCollidable);
                  }
               }
            }
         }

         if (robotHasArms)
         {
            armManager.create(baseUI);
            for (RobotSide side : interactableHands.sides())
            {
               // TODO this should probably not handle the space event!
               // This sends a command to the controller.
               interactableHands.get(side).setActionExecutor(armManager.getSubmitDesiredArmSetpointsCallback(side));
               interactableHands.get(side).setOpenHand(() -> armManager.getHandManager().publishHandCommand(side, HandConfiguration.OPEN));
               interactableHands.get(side).setCloseHand(() -> armManager.getHandManager().publishHandCommand(side, HandConfiguration.CLOSE));
               interactableHands.get(side).setGotoDoorAvoidanceArmAngles(() -> armManager.executeDoorAvoidanceArmAngles(side));
               interactableHands.get(side).setGotoArmHome(() -> armManager.executeArmHome(side));
            }
         }

         baseUI.getVRManager().getContext().addVRPickCalculator(this::calculateVRPick);
         baseUI.getVRManager().getContext().addVRInputProcessor(this::processVRInput);
         baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(this::calculate3DViewPick);
         baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::process3DViewInput);
         baseUI.getPrimary3DPanel().addImGuiOverlayAddition(this::renderTooltipsAndContextMenus);
         interactablesEnabled.set(true);

         demoPoses = new RDXHumanoidDemoPoses(robotModel, syncedRobot, ros2Helper, teleoperationParameters);
         addChild(demoPoses);
      }

      RDX3DPanelToolbarButton standPrepButton = baseUI.getPrimary3DPanel().addToolbarButton();
      standPrepButton.loadAndSetIcon("icons/standPrep.png");
      standPrepButton.setOnPressed(robotLowLevelMessenger::sendStandRequest);
      standPrepButton.setTooltipText("Stand prep");

      RDX3DPanelToolbarButton freezeButton = baseUI.getPrimary3DPanel().addToolbarButton();
      freezeButton.loadAndSetIcon("icons/freeze.png");
      freezeButton.setTooltipText("Freeze");
      freezeButton.setOnPressed(robotLowLevelMessenger::sendFreezeRequest);

      RDX3DPanelToolbarButton abortToolbarButton = baseUI.getPrimary3DPanel().addToolbarButton();
      abortToolbarButton.loadAndSetIcon("icons/abort.png");
      abortToolbarButton.setTooltipText("Abort");
      abortToolbarButton.setOnPressed(locomotionManager::sendAbortWalkingMessage);

      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
   }

   public void update()
   {
      syncedRobot.update();
      desiredRobot.update();

      locomotionManager.update();

      if (interactablesEnabled.get())
      {
         locomotionManager.updateWalkPathControlRing();

         if (interactablesAvailable)
         {
            if (robotHasArms)
            {
               armManager.update();
               
               boolean handInteractablesAreDeleted = true;
               for (RobotSide side : interactableHands.sides())
               {
                  handInteractablesAreDeleted &= interactableHands.get(side).isDeleted();
               }
               desiredRobot.setActive(!handInteractablesAreDeleted);

               if (!handInteractablesAreDeleted)
               {
                  for (RobotSide side : interactableHands.sides())
                  {
                     desiredRobot.setArmShowing(side, !interactableHands.get(side).isDeleted() 
                        && (armManager.getArmControlMode() == RDXArmControlMode.JOINTSPACE || armManager.getArmControlMode() == RDXArmControlMode.HYBRID));
                     desiredRobot.setArmColor(side, RDXIKSolverColors.getColor(armManager.getArmIKSolvers().get(side).getQuality()));
                  }
               }
            }

            for (RobotSide side : interactableFeet.sides())
            {
               if (interactableFeet.get(side).getBecomesModified().poll())
               {
                  locomotionManager.setLegControlModeToSingleSupportFootPosing();
               }
            }

            avoidanceCollisionModel.update();
            contactCollisionModel.update();

            for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
               robotPartInteractable.update();
         }
      }

      boolean allAreDeleted = true;
      if (interactablesAvailable)
      {
         allAreDeleted &= interactableChest.isDeleted() && interactablePelvis.isDeleted();
         if (robotHasArms)
         {
            for (RobotSide side : interactableHands.sides())
            {
               allAreDeleted &= interactableHands.get(side).isDeleted();
            }
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
      if (interactablesEnabled.get())
      {
         locomotionManager.calculateWalkPathControlRingVRPick(vrContext);

         if (interactablesAvailable && showContactCollisionMeshes.get())
            contactCollisionModel.calculateVRPick(vrContext);

         for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
            robotPartInteractable.calculateVRPick(vrContext);
      }
   }

   private void processVRInput(RDXVRContext vrContext)
   {
      if (interactablesAvailable)
      {
         locomotionManager.processWalkPathControlRingVRInput(vrContext);

         if (interactablesEnabled.get() && showContactCollisionMeshes.get())
            contactCollisionModel.processVRInput(vrContext);

         for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
            robotPartInteractable.processVRInput(vrContext);
      }
   }

   private void calculate3DViewPick(ImGui3DViewInput input)
   {
      if (interactablesEnabled.get())
      {
         locomotionManager.calculateWalkPathControlRing3DViewPick(input);

         if (interactablesAvailable)
         {
            if (input.isWindowHovered() && showContactCollisionMeshes.get())
               contactCollisionModel.calculate3DViewPick(input);

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
         locomotionManager.processWalkPathControlRing3dViewInput(input);

         if (interactablesAvailable)
         {
            if (showContactCollisionMeshes.get())
               contactCollisionModel.process3DViewInput(input);

            interactableChest.process3DViewInput(input);
            interactablePelvis.process3DViewInput(input);

            for (RobotSide side : interactableFeet.sides())
            {
               if (interactableFeet.get(side).process3DViewInput(input))
               {
                  locomotionManager.setLegControlModeToSingleSupportFootPosing();
               }
            }

            if (robotHasArms)
            {
               for (RobotSide side : interactableHands.sides())
               {
                  interactableHands.get(side).process3DViewInput(input);
               }
            }
         }
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.pushFont(ImGuiTools.getMediumFont());
      ImGui.text("Whole Body");
      ImGui.popFont();

      robotLowLevelMessenger.renderImGuiWidgets();

      pelvisHeightSlider.renderImGuiWidgets();
      chestPitchSlider.renderImGuiWidgets();
      chestYawSlider.renderImGuiWidgets();

      trajectoryTimeSlider.renderImGuiWidget();

      if (ImGui.button(labels.get("Delete all Interactables")) || ImGui.getIO().getKeyShift() && ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
      {
         clearInteractablesAndLocomotionGraphics();
      }
      ImGuiTools.previousWidgetTooltip("Shift + Escape");
      ImGui.sameLine();
      if (interactablesAvailable)
      {
         ImGui.checkbox("Interactables Enabled", interactablesEnabled);
      }

      if (ImGui.collapsingHeader(labels.get("Interactable Selections"), interactableSelections))
      {
         ImGui.indent();
         if (interactablesAvailable)
         {
            ImGui.text("Chest:");
            ImGuiTools.previousWidgetTooltip("Send with: Spacebar");
            ImGui.sameLine();
            interactableChest.renderImGuiWidgets();

            ImGui.text("Pelvis:");
            ImGuiTools.previousWidgetTooltip("Send with: Spacebar");
            ImGui.sameLine();
            interactablePelvis.renderImGuiWidgets();

            if (robotHasArms)
            {
               for (RobotSide side : interactableHands.sides())
               {
                  ImGui.text(side.getPascalCaseName() + " Hand:");
                  ImGui.sameLine();
                  interactableHands.get(side).renderImGuiWidgets();
               }
            }

            for (RobotSide side : interactableFeet.sides())
            {
               ImGui.text(side.getPascalCaseName() + " Foot:");
               ImGui.sameLine();
               interactableFeet.get(side).renderImGuiWidgets();
            }
         }
         ImGui.unindent();
      }

      ImGui.text("Show collisions:");
      ImGui.sameLine();
      ImGui.checkbox("Contact", showContactCollisionMeshes);
      ImGui.sameLine();
      ImGui.checkbox("Avoidance", showAvoidanceCollisionMeshes);

      // TODO: Add transparency sliders
      // TODO: Add motion previews

      ImGui.pushFont(ImGuiTools.getMediumFont());
      ImGui.text("Locomotion");
      ImGui.popFont();
      locomotionManager.renderImGuiWidgets();

      ImGui.pushFont(ImGuiTools.getMediumFont());
      ImGui.text("Arms & Hands");
      ImGui.popFont();
      armManager.renderImGuiWidgets();
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

   // The create method adds the renderables, so this shouldn't be accessed externally.
   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         desiredRobot.getRenderables(renderables, pool, sceneLevels);

         if (showGraphics.get())
         {
            locomotionManager.getRenderables(renderables, pool);
         }

         if (interactablesEnabled.get())
         {
            if (interactablesAvailable)
            {
               if (showAvoidanceCollisionMeshes.get())
                  avoidanceCollisionModel.getRenderables(renderables, pool);
               if (showContactCollisionMeshes.get())
                  contactCollisionModel.getRenderables(renderables, pool);

               for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
                  robotPartInteractable.getVirtualRenderables(renderables, pool);
            }

            locomotionManager.getWalkPathControlRingVirtualRenderables(renderables, pool);
         }
      }
   }

   public void clearInteractablesAndLocomotionGraphics()
   {
      locomotionManager.deleteAll();

      for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
         robotPartInteractable.delete();
   }
   public void destroy()
   {
      desiredRobot.destroy();
      locomotionManager.destroy();
   }

   public ImBoolean getInteractablesEnabled()
   {
      return interactablesEnabled;
   }

   public RDXRobotCollisionModel getAvoidanceCollisionModel()
   {
      return avoidanceCollisionModel;
   }

   public RDXLocomotionParameters getLocomotionParameters()
   {
      return locomotionManager.getLocomotionParameters();
   }

   public RDXLocomotionManager getLocomotionManager()
   {
      return locomotionManager;
   }

   public RDXArmManager getArmManager()
   {
      return armManager;
   }
}
