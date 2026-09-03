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
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.LocomotionParameters;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetDoubleWidget;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.affordances.RDXArmManager;
import us.ihmc.rdx.ui.affordances.RDXInteractableFoot;
import us.ihmc.rdx.ui.affordances.RDXInteractableHand;
import us.ihmc.rdx.ui.affordances.RDXInteractableRobotLink;
import us.ihmc.rdx.ui.affordances.RDXInteractableTools;
import us.ihmc.rdx.ui.affordances.RDXPelvisControlMode;
import us.ihmc.rdx.ui.affordances.RDXRobotCollidable;
import us.ihmc.rdx.ui.collidables.RDXRobotCollisionModel;
import us.ihmc.rdx.ui.hands.RDXHandInterface.HandAction;
import us.ihmc.rdx.ui.hands.RDXHandManager;
import us.ihmc.rdx.ui.interactable.RDXHumanoidDoFsWidgets;
import us.ihmc.rdx.ui.interactable.RDXPelvisHeightSlider;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionManager;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
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
 * <li>{@link RDXHandManager Hand configuration manager} - lives inside the arm manager</li>
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
   private final SideDependentList<Boolean> hasArms = new SideDependentList<>();
   private boolean hasEitherArm = false;
   private boolean hasHead = false;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final RDXTeleoperationParameters teleoperationParameters;
   private final RDXStoredPropertySetTuner teleoperationParametersTuner = new RDXStoredPropertySetTuner("Teleoperation Parameters");
   private final RDXHardwareControlStateManager hardwareControlStateManager;

   private final RDXPelvisHeightSlider pelvisHeightSlider;
   private final RDXHumanoidDoFsWidgets dofsWidgets;
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
   private RDXInteractableRobotLink interactableHead;
   private final ArrayList<RDXInteractableRobotLink> allInteractableRobotLinks = new ArrayList<>();
   private final ImString tempImGuiText = new ImString(1000);
   private final ImBoolean interactableSelections = new ImBoolean(true);
   private final boolean interactablesAvailable;
   private final RDXWholeBodyIKManager wholeBodyIKManager;
   private ImGuiStoredPropertySetDoubleWidget trajectoryTimeSlider;
   private RDXHumanoidDemoPoses demoPoses;

   /** This tracker should be shared with the sub-managers to keep the state consistent. */
   private final ControllerStatusTracker controllerStatusTracker;
   private final LogToolsLogger logToolsLogger = new LogToolsLogger();

   /** For post-processing WBMPC commands */
   private RDXPelvisControlMode pelvisControlMode = RDXPelvisControlMode.PELVIS;
   private final FramePoint3D tempCurrentCoMPosition = new FramePoint3D();
   private final FramePoint3D tempDesiredPelvisPosition = new FramePoint3D();
   private final FramePoint3D tempCurrentPelvisPosition = new FramePoint3D();
   private static final double HEIGHT_TOLERANCE = 0.01;

   /**
    * For use without interactables available. May crash if a YoVariableClient is needed.
    */
   public RDXTeleoperationManager(ROS2SyncedRobotModel syncedRobot, ROS2ControllerHelper controllerHelper)
   {
      this(syncedRobot, controllerHelper, null, null, null);
   }

   /**
    * Enable interactables and use a YoVariable client to show wrist force arrows on
    * some robots.
    */
   public RDXTeleoperationManager(ROS2SyncedRobotModel syncedRobot,
                                  ROS2ControllerHelper controllerHelper,
                                  RobotCollisionModel robotSelfCollisionModel,
                                  RobotCollisionModel robotSelectionCollisionModel,
                                  YoVariableClientHelper yoVariableClientHelper)
   {
      this(syncedRobot, controllerHelper, robotSelfCollisionModel, robotSelectionCollisionModel, yoVariableClientHelper, new RDXHardwareControlStateManager(controllerHelper));
   }

   public RDXTeleoperationManager(ROS2SyncedRobotModel syncedRobot,
                                  ROS2ControllerHelper controllerHelper,
                                  RobotCollisionModel robotSelfCollisionModel,
                                  RobotCollisionModel robotSelectionCollisionModel,
                                  YoVariableClientHelper yoVariableClientHelper,
                                  RDXHardwareControlStateManager hardwareControlStateManager)
   {
      super("Teleoperation");

      this.hardwareControlStateManager = hardwareControlStateManager;

      setRenderMethod(this::renderImGuiWidgets);

      this.syncedRobot = syncedRobot;

      addChild(teleoperationParametersTuner);
      robotModel = controllerHelper.getRobotModel();
      hasHead = robotModel.getRobotVersion().hasHead();
      for (RobotSide side : RobotSide.values)
      {
         boolean hasArm = robotModel.getRobotVersion().hasArm(side);
         hasEitherArm |= hasArm;
         hasArms.put(side, hasArm);
      }
      ros2Helper = controllerHelper;
      this.yoVariableClientHelper = yoVariableClientHelper;

      teleoperationParameters = new RDXTeleoperationParameters(robotModel.getSimpleRobotName());
      teleoperationParameters.load();

      desiredRobot = new RDXDesiredRobot(robotModel);
      desiredRobot.setSceneLevels(RDXSceneLevel.VIRTUAL);

      controllerStatusTracker = new ControllerStatusTracker(logToolsLogger, ros2Helper.getROS2Node(), syncedRobot);

      locomotionManager = new RDXLocomotionManager(robotModel, controllerHelper, syncedRobot, controllerStatusTracker, this);

      interactablesAvailable = robotSelfCollisionModel != null;
      if (interactablesAvailable)
      {
         avoidanceCollisionModel = new RDXRobotCollisionModel(robotSelfCollisionModel);
         contactCollisionModel = new RDXRobotCollisionModel(robotSelectionCollisionModel);
      }

      wholeBodyIKManager = new RDXWholeBodyIKManager(robotModel, teleoperationParameters, desiredRobot, ros2Helper, syncedRobot, controllerStatusTracker);

      // create the manager for the desired arm setpoints
      armManager = new RDXArmManager(controllerHelper,
                                     robotModel,
                                     syncedRobot,
                                     desiredRobot,
                                     teleoperationParameters,
                                     interactableHands,
                                     wholeBodyIKManager::getEnabled);

      pelvisHeightSlider = new RDXPelvisHeightSlider(syncedRobot, ros2Helper, teleoperationParameters);
      dofsWidgets = new RDXHumanoidDoFsWidgets(syncedRobot, ros2Helper, teleoperationParameters);

      RDXBaseUI.getInstance().getKeyBindings().register("Delete all Interactables", "Shift + Escape");
   }

   public void create(RDXBaseUI baseUI)
   {
      this.baseUI = baseUI;
      desiredRobot.create();
      dofsWidgets.getDesiredRobot().create();

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
                                           robotModel.getChestGraphicToFrameTransform(),
                                           new RigidBodyTransform(),
                                           modelFileName,
                                           baseUI.getPrimary3DPanel());
                  interactableChest.setActionExecutor(() ->
                                                      {
                                                         if (!wholeBodyIKManager.getEnabled())
                                                         {
                                                            RDXBaseUI.pushNotification("Commanding chest trajectory...");
                                                            processChestCommand();
                                                         }
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
                  interactablePelvis.create(robotCollidable, syncedRobot.getReferenceFrames().getPelvisFrame(), modelFileName, baseUI.getPrimary3DPanel());
                  interactablePelvis.setActionExecutor(() ->
                                                       {
                                                          if (!wholeBodyIKManager.getEnabled())
                                                          {
                                                             if (pelvisControlMode == RDXPelvisControlMode.COM)
                                                                processCoMDirectCommand();
                                                             else
                                                                processPelvis6DCommand();
                                                          }
                                                       });
                  allInteractableRobotLinks.add(interactablePelvis);
               }
               else
               {
                  interactablePelvis.addAdditionalRobotCollidable(robotCollidable);
               }
            }
            if (hasHead)
            {
               if (robotCollidable.getRigidBodyName().equals(fullRobotModel.getHead().getName()))
               {
                  if (interactableHead == null)
                  {
                     interactableHead = new RDXInteractableRobotLink();
                     interactableHead.create(robotCollidable,
                                             syncedRobot.getReferenceFrames().getHeadFrame(),
                                             RDXInteractableTools.getModelGraphicToBodyTransform(robotDefinition.getRigidBodyDefinition(robotCollidable.getRigidBodyName())),
                                             new RigidBodyTransform(),
                                             modelFileName,
                                             baseUI.getPrimary3DPanel());
                     interactableHead.setActionExecutor(() ->
                                                        {
                                                           if (!wholeBodyIKManager.getEnabled())
                                                           {
                                                              processHeadCommand();
                                                           }
                                                        });
                     allInteractableRobotLinks.add(interactableHead);
                  }
                  else
                  {
                     interactableHead.addAdditionalRobotCollidable(robotCollidable);
                  }
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
                                                        {
                                                           if (!wholeBodyIKManager.getEnabled())
                                                           {
                                                              RDXBaseUI.pushNotification("Commanding foot trajectory...");
                                                              FramePose3D afterAnklePose = new FramePose3D();
                                                              afterAnklePose.setToZero(interactableFoot.getLinkFrame());
                                                              afterAnklePose.changeFrame(ReferenceFrame.getWorldFrame());
                                                              ros2Helper.publishToController(HumanoidMessageTools.createFootTrajectoryMessage(side,
                                                                                                                                              teleoperationParameters.getTrajectoryTime(),
                                                                                                                                              afterAnklePose));
                                                           }
                                                        });
                     interactableFeet.put(side, interactableFoot);
                     allInteractableRobotLinks.add(interactableFoot);
                  }
                  else
                  {
                     interactableFeet.get(side).addAdditionalRobotCollidable(robotCollidable);
                  }
               }
               if (hasArms.get(side) && RDXInteractableHand.robotCollidableIsHand(side, robotCollidable, fullRobotModel))
               {
                  if (!interactableHands.containsKey(side))
                  {
                     RDXInteractableHand interactableHand = new RDXInteractableHand(side,
                                                                                    baseUI,
                                                                                    robotCollidable,
                                                                                    robotModel,
                                                                                    syncedRobot,
                                                                                    yoVariableClientHelper);
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

         if (hasEitherArm)
         {
            armManager.create(baseUI);
            for (RobotSide side : interactableHands.sides())
            {
               interactableHands.get(side).setActionExecutor(() ->
                                                             {
                                                                if (!wholeBodyIKManager.getEnabled())
                                                                {
                                                                   armManager.executeDesiredArmCommand(side);
                                                                }
                                                             });
               interactableHands.get(side).setOpenHand(() -> armManager.getHandManager().getHand(side).sendCommand(HandAction.OPEN));
               interactableHands.get(side).setCloseHand(() -> armManager.getHandManager().getHand(side).sendCommand(HandAction.CLOSE));
               interactableHands.get(side).setGotoArmHome(() -> armManager.executeArmHome(side));
            }
         }

         wholeBodyIKManager.setInteractables(interactableHands, interactableFeet, interactableChest, interactablePelvis);

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
      standPrepButton.setOnPressed(hardwareControlStateManager::sendStandPrepRequest);
      standPrepButton.setTooltipText("Stand prep");

      RDX3DPanelToolbarButton freezeButton = baseUI.getPrimary3DPanel().addToolbarButton();
      freezeButton.loadAndSetIcon("icons/freeze.png");
      freezeButton.setTooltipText("Freeze");
      freezeButton.setOnPressed(hardwareControlStateManager::sendFreezeRequest);

      RDX3DPanelToolbarButton abortToolbarButton = baseUI.getPrimary3DPanel().addToolbarButton();
      abortToolbarButton.loadAndSetIcon("icons/abort.png");
      abortToolbarButton.setTooltipText("Abort");
      abortToolbarButton.setOnPressed(locomotionManager::sendAbortWalkingMessage);

      clearInteractablesAndLocomotionGraphics();
      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
   }

   public void update()
   {
      hardwareControlStateManager.update();
      desiredRobot.update();
      dofsWidgets.getDesiredRobot().update();

      locomotionManager.update();
      armManager.update(interactablesEnabled.get());

      if (interactablesEnabled.get())
      {
         locomotionManager.updateWalkPathControlRing();

         if (interactablesAvailable)
         {
            for (RobotSide side : interactableHands.sides())
               desiredRobot.setArmShowing(side, wholeBodyIKManager.getEnabled());
            for (RobotSide side : interactableFeet.sides())
               desiredRobot.setLegShowing(side, wholeBodyIKManager.getEnabled());
            desiredRobot.setChestShowing(wholeBodyIKManager.getEnabled());
            desiredRobot.setPelvisShowing(wholeBodyIKManager.getEnabled());

            if (wholeBodyIKManager.getEnabled())
            {
               wholeBodyIKManager.update();

               // So the arm IK solvers will solve when whole body is deselected
               for (RobotSide side : RobotSide.values)
                  armManager.getArmIKSolvers().get(side).reset();
            }
            else if (hasEitherArm)
            {
               // So the whole body IK will solve when selected
               wholeBodyIKManager.reset();

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
                     desiredRobot.setArmShowing(side, !interactableHands.get(side).isDeleted());
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
         if (hasHead)
            allAreDeleted &= interactableHead.isDeleted();
         for (RobotSide side : interactableHands.sides())
            allAreDeleted &= interactableHands.get(side).isDeleted();
         for (RobotSide side : interactableFeet.sides())
            allAreDeleted &= interactableFeet.get(side).isDeleted();
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

   /**
    * Function that routes the corresponding height displacement as CoM Trajectory Command, and the orientation
    * as a Pelvis Trajectory Command
    *
    * @comment: For now, it can only process z displacements, it will be easy to extend it's funcionality for
    *       displacements in the other directions
    * @comment: Investigate which cases causes it to freak out
    */
   private void processCoMDirectCommand()
   {
      FramePose3DReadOnly desiredOrientation = interactablePelvis.getPose();

      // Get all positions in world frame
      tempCurrentCoMPosition.setToZero(syncedRobot.getReferenceFrames().getCenterOfMassFrame());
      tempCurrentCoMPosition.changeFrame(syncedRobot.getReferenceFrames().getWorldFrame());

      tempDesiredPelvisPosition.setIncludingFrame(desiredOrientation.getPosition());
      tempDesiredPelvisPosition.changeFrame(syncedRobot.getReferenceFrames().getWorldFrame());

      tempCurrentPelvisPosition.setToZero(syncedRobot.getReferenceFrames().getPelvisFrame());
      tempCurrentPelvisPosition.changeFrame(syncedRobot.getReferenceFrames().getWorldFrame());

      // Calculate desired CoM height maintaining current offset
      double currentComToPelvisOffset = tempCurrentCoMPosition.getZ() - tempCurrentPelvisPosition.getZ();
      double desiredCoMHeight = tempDesiredPelvisPosition.getZ() + currentComToPelvisOffset;
      double deltaZ = Math.abs(desiredCoMHeight - tempCurrentCoMPosition.getZ());

      if (deltaZ > HEIGHT_TOLERANCE)
      {
         // Send center of mass height trajectory
         RDXBaseUI.pushNotification("Commanding CoM height trajectory...");
         ros2Helper.publishToController(HumanoidMessageTools.createCenterOfMassTrajectoryMessage(teleoperationParameters.getTrajectoryTime(),
                                                                                                 new Point3D(tempCurrentCoMPosition.getX(),
                                                                                                             tempCurrentCoMPosition.getY(),
                                                                                                             desiredCoMHeight)));
      }
      else
      {
         // Send orientation-only trajectory for other movements
         RDXBaseUI.pushNotification("Commanding pelvis trajectory...");
         ros2Helper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(teleoperationParameters.getTrajectoryTime(), desiredOrientation));
      }
   }

   private void processPelvis6DCommand()
   {
      RDXBaseUI.pushNotification("Commanding pelvis  trajectory...");
      ros2Helper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(teleoperationParameters.getTrajectoryTime(),
                                                                                        interactablePelvis.getPose()));
   }

   private void processChestCommand()
   {
      RDXBaseUI.pushNotification("Commanding chest trajectory...");
      SpineJointName[] spineJointNamesArray = syncedRobot.getRobotModel().getJointMap().getSpineJointNames();
      double[] desiredSpineJointValues = new double[spineJointNamesArray.length];

      FramePose3D chestInPelvisFrame = new FramePose3D(interactableChest.getPose());
      chestInPelvisFrame.changeFrame(syncedRobot.getReferenceFrames().getPelvisFrame());

      double desiredYaw = chestInPelvisFrame.getYaw();
      double desiredPitch = chestInPelvisFrame.getPitch();
      double desiredRoll = chestInPelvisFrame.getRoll();

      for (int i = 0; i < spineJointNamesArray.length; i++)
      {
         switch (spineJointNamesArray[i])
         {
            case SPINE_YAW:
               desiredSpineJointValues[i] = desiredYaw;
               break;
            case SPINE_PITCH:
               desiredSpineJointValues[i] = desiredPitch;
               break;
            case SPINE_ROLL:
               desiredSpineJointValues[i] = desiredRoll;
               break;
            default:
               desiredSpineJointValues[i] = 0.0; // fallback
         }
      }
      ros2Helper.publishToController(HumanoidMessageTools.createChestHybridTrajectoryMessage(interactableChest.getPoseGizmo().getGizmoFrame(),
                                                                                             syncedRobot.getReferenceFrames().getPelvisFrame(),
                                                                                             desiredSpineJointValues,
                                                                                             teleoperationParameters.getTrajectoryTime()));
   }

   private void processHeadCommand()
   {
      RDXBaseUI.pushNotification("Commanding head trajectory...");
      NeckJointName[] neckJointNamesArray = syncedRobot.getRobotModel().getJointMap().getNeckJointNames();
      double[] desiredNeckJointValues = new double[neckJointNamesArray.length];

      FramePose3D headInChestFrame = new FramePose3D(interactableHead.getPose());
      headInChestFrame.changeFrame(syncedRobot.getReferenceFrames().getChestFrame());

      double desiredYaw = headInChestFrame.getYaw();
      double desiredPitch = headInChestFrame.getPitch();
      double desiredRoll = headInChestFrame.getRoll();

      for (int i = 0; i < neckJointNamesArray.length; i++)
      {
         switch (neckJointNamesArray[i])
         {
            case DISTAL_NECK_YAW:
               desiredNeckJointValues[i] = desiredYaw;
               break;
            case DISTAL_NECK_PITCH:
               desiredNeckJointValues[i] = desiredPitch;
               break;
            case DISTAL_NECK_ROLL:
               desiredNeckJointValues[i] = desiredRoll;
               break;
            default:
               desiredNeckJointValues[i] = 0.0; // fallback
         }
      }

      ros2Helper.publishToController(HumanoidMessageTools.createHeadHybridTrajectoryMessage(interactableHead.getPoseGizmo().getGizmoFrame(),
                                                                                            syncedRobot.getReferenceFrames().getChestFrame(),
                                                                                            desiredNeckJointValues,
                                                                                            teleoperationParameters.getTrajectoryTime()));
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
            if (hasHead)
               interactableHead.process3DViewInput(input);

            for (RobotSide side : interactableFeet.sides())
            {
               if (interactableFeet.get(side).process3DViewInput(input))
               {
                  locomotionManager.setLegControlModeToSingleSupportFootPosing();
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
      hardwareControlStateManager.renderImGuiWidgets(syncedRobot.getRobotModel(),
                                                     syncedRobot.getReferenceFrames(),
                                                     armManager,
                                                     teleoperationParameters.getPelvisMaximumHeight());
      trajectoryTimeSlider.renderImGuiWidget();
      renderWholeBodyWidgets();
      locomotionManager.renderImGuiWidgets();
      armManager.renderImGuiWidgets();
   }

   private void renderWholeBodyWidgets()
   {
      if (ImGui.collapsingHeader(labels.get("Whole-Body")))
      {
         float widgetStartX = 168.0f;
         ImGui.text("Joint Position Control:");
         ImGui.sameLine();
         ImGui.setCursorPosX(widgetStartX);
         dofsWidgets.renderImGuiWidgets();

         ImGui.separator();
         ImGui.text("Pelvis Height:");
         ImGui.sameLine();
         ImGui.setCursorPosX(widgetStartX);
         ImGui.setNextItemWidth(-1);
         pelvisHeightSlider.renderImGuiWidgets();

         ImGui.text("Pelvis Height Control Mode:");
         ImGui.sameLine();
         ImGui.setCursorPosX(widgetStartX);
         if (ImGui.radioButton(labels.get("Pelvis"), pelvisControlMode == RDXPelvisControlMode.PELVIS))
         {
            pelvisControlMode = RDXPelvisControlMode.PELVIS;
         }
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("CoM"), pelvisControlMode == RDXPelvisControlMode.COM))
         {
            pelvisControlMode = RDXPelvisControlMode.COM;
         }
         ImGui.separator();
         ImGui.text("3D Interactables: ");
         ImGui.sameLine();
         ImGui.setCursorPosX(widgetStartX);
         if (interactablesAvailable)
         {
            ImGui.checkbox("Enable", interactablesEnabled);
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Delete all")) || ImGui.getIO().getKeyShift() && ImGui.isKeyPressed(ImGuiTools.getEscapeKey()))
         {
            clearInteractablesAndLocomotionGraphics();
         }
         ImGuiTools.previousWidgetTooltip("Shift + Escape");
         ImGui.setCursorPosX(widgetStartX);
         wholeBodyIKManager.renderImGuiWidgets();

         ImGui.setCursorPosX(widgetStartX);
         if (ImGui.collapsingHeader(labels.get("Interactable Selections"), interactableSelections))
         {
            ImGui.indent();
            if (interactablesAvailable)
            {
               float radioStartX = 100.0f;
               if (hasHead)
               {
                  ImGui.text("Head:");
                  ImGuiTools.previousWidgetTooltip("Send with: Spacebar");
                  ImGui.sameLine();
                  ImGui.setCursorPosX(radioStartX);
                  interactableHead.renderImGuiWidgets();
               }

               ImGui.text("Chest:");
               ImGuiTools.previousWidgetTooltip("Send with: Spacebar");
               ImGui.sameLine();
               ImGui.setCursorPosX(radioStartX);
               interactableChest.renderImGuiWidgets();

               ImGui.text("Pelvis:");
               ImGuiTools.previousWidgetTooltip("Send with: Spacebar");
               ImGui.sameLine();
               ImGui.setCursorPosX(radioStartX);
               interactablePelvis.renderImGuiWidgets();

               for (RobotSide side : interactableHands.sides())
               {
                  ImGui.text(side.getPascalCaseName() + " Hand:");
                  ImGui.sameLine();
                  ImGui.setCursorPosX(radioStartX);
                  interactableHands.get(side).renderImGuiWidgets();
               }

               for (RobotSide side : interactableFeet.sides())
               {
                  ImGui.text(side.getPascalCaseName() + " Foot:");
                  ImGui.sameLine();
                  ImGui.setCursorPosX(radioStartX);
                  interactableFeet.get(side).renderImGuiWidgets();
               }
            }
            ImGui.unindent();
         }
         ImGui.separator();
         ImGui.text("Collisions: ");
         ImGui.sameLine();
         ImGui.setCursorPosX(widgetStartX);
         ImGui.checkbox("Show contact", showContactCollisionMeshes);
         ImGui.sameLine();
         ImGui.checkbox("Show avoidance", showAvoidanceCollisionMeshes);
      }
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
            ImGui.inputTextMultiline(labels.getHidden(side.getPascalCaseName() + "DesiredRobotJointAngles"),
                                     tempImGuiText,
                                     0,
                                     60,
                                     ImGuiInputTextFlags.ReadOnly);

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
         if (fullRobotModel.getArmJoint(side, armJoint) != null)
         {
            double q = fullRobotModel.getArmJoint(side, armJoint).getQ();
            jointAnglesString.append(FormattingTools.getFormattedDecimal3D(q));
         }
         else
         {
            jointAnglesString.append("0.0");
         }
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
         dofsWidgets.getDesiredRobot().getRenderables(renderables, pool, sceneLevels);

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
      dofsWidgets.reset();

      for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
         robotPartInteractable.delete();
   }

   public void destroy()
   {
      hardwareControlStateManager.destroy();
      desiredRobot.destroy();
      locomotionManager.destroy();
      armManager.destroy();
      dofsWidgets.destroy();
   }

   public ImBoolean getInteractablesEnabled()
   {
      return interactablesEnabled;
   }

   public RDXRobotCollisionModel getAvoidanceCollisionModel()
   {
      return avoidanceCollisionModel;
   }

   public LocomotionParameters getLocomotionParameters()
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
