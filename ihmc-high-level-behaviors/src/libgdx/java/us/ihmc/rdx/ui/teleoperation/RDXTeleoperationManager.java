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
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetDoubleWidget;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.*;
import us.ihmc.rdx.ui.collidables.RDXRobotCollisionModel;
import us.ihmc.rdx.ui.interactable.RDXChestOrientationSlider;
import us.ihmc.rdx.ui.interactable.RDXPelvisHeightSlider;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionManager;
import us.ihmc.rdx.ui.teleoperation.locomotion.RDXLocomotionParameters;
import us.ihmc.rdx.ui.visualizers.RDXVisualizer;
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
import java.util.List;
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
 * <li>{@link RDXHandConfigurationManager Hand manager} TODO: Perhaps this should go in arm manager?</li>
 * <li>{@link RDXArmManager Arm manager}</li>
 * <li>{@link RDXLocomotionManager Locomotion manager}</li>
 * </ul>
 *
 * TODO:
 * <ul>
 * <li>Possibly extract simple controller controls to a smaller panel class, like remote safety controls or something.</li>
 * </ul>
 */
public class RDXTeleoperationManager extends ImGuiPanel
{
   RDXBaseUI baseUI;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CommunicationHelper communicationHelper;
   private final ROS2ControllerHelper ros2Helper;
   private final YoVariableClientHelper yoVariableClientHelper;
   private final DRCRobotModel robotModel;
   private final boolean robotHasArms;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final RDXTeleoperationParameters teleoperationParameters;
   private final ImGuiStoredPropertySetTuner teleoperationParametersTuner = new ImGuiStoredPropertySetTuner("Teleoperation Parameters");
   private final RDXRobotLowLevelMessenger robotLowLevelMessenger;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final AStarBodyPathPlannerParametersBasics bodyPathPlannerParameters;
   private final ImGuiStoredPropertySetTuner footstepPlanningParametersTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (Teleoperation)");
   private final ImGuiStoredPropertySetTuner bodyPathPlanningParametersTuner = new ImGuiStoredPropertySetTuner("Body Path Planner Parameters (Teleoperation)");

   private final RDXPelvisHeightSlider pelvisHeightSlider;
   private final RDXChestOrientationSlider chestPitchSlider;
   private final RDXChestOrientationSlider chestYawSlider;
   private final RDXDesiredRobot desiredRobot;
   private final RDXHandConfigurationManager handManager;
   private RDXRobotCollisionModel selfCollisionModel;
   private RDXRobotCollisionModel environmentCollisionModel;
   private RDXArmManager armManager;
   private final RDXLocomotionManager locomotionManager;
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
   private final boolean interactablesAvailable;
   private ImGuiStoredPropertySetDoubleWidget trajectoryTimeSlider;

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
                                  RobotCollisionModel robotEnvironmentCollisionModel,
                                  YoVariableClientHelper yoVariableClientHelper)
   {
      super("Teleoperation");

      setRenderMethod(this::renderImGuiWidgets);
      addChild(teleoperationParametersTuner);
      addChild(footstepPlanningParametersTuner);
      addChild(bodyPathPlanningParametersTuner);
      this.communicationHelper = communicationHelper;
      robotModel = communicationHelper.getRobotModel();
      robotHasArms = robotModel.getRobotVersion().hasArms();
      ros2Helper = communicationHelper.getControllerHelper();
      this.yoVariableClientHelper = yoVariableClientHelper;

      this.footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      this.bodyPathPlannerParameters = robotModel.getAStarBodyPathPlannerParameters();

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

      locomotionManager = new RDXLocomotionManager(robotModel, communicationHelper, syncedRobot, ros2Helper, controllerStatusTracker);
      addChild(locomotionManager.getSwingFootPlanningParametersTuner());

      interactablesAvailable = robotSelfCollisionModel != null;
      if (interactablesAvailable)
      {
         selfCollisionModel = new RDXRobotCollisionModel(robotSelfCollisionModel);
         environmentCollisionModel = new RDXRobotCollisionModel(robotEnvironmentCollisionModel);
         if (robotHasArms)
         {
            armManager = new RDXArmManager(robotModel, syncedRobot, desiredRobot.getDesiredFullRobotModel(), ros2Helper, teleoperationParameters);
         }
      }

      if (robotHasArms)
         handManager = new RDXHandConfigurationManager();
      else
         handManager = null;
   }

   public void create(RDXBaseUI baseUI)
   {
      this.baseUI = baseUI;
      desiredRobot.create();

      locomotionManager.create(baseUI);
      addChild(locomotionManager.getLocomotionParametersTuner());

      footstepPlanningParametersTuner.create(footstepPlannerParameters, false, () -> locomotionManager.setFootstepPlannerParameters(footstepPlannerParameters));
      bodyPathPlanningParametersTuner.create(bodyPathPlannerParameters, false, () -> locomotionManager.setBodyPathPlannerParameters(bodyPathPlannerParameters));

      teleoperationParametersTuner.create(teleoperationParameters);

      trajectoryTimeSlider = teleoperationParametersTuner.createDoubleSlider(RDXTeleoperationParameters.trajectoryTime, 0.1, 0.5, "s", "%.2f");

      if (interactablesAvailable)
      {
         selfCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkGreen(), 0.4));
         environmentCollisionModel.create(syncedRobot, YoAppearanceTools.makeTransparent(YoAppearance.DarkRed(), 0.4));

         // create the manager for the desired arm setpoints
         if (armManager != null)
            armManager.create(baseUI);

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
               if (robotHasArms && RDXInteractableHand.robotCollidableIsHand(side, robotCollidable, fullRobotModel))
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

      if (robotHasArms)
         handManager.create(baseUI, communicationHelper);

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
               armManager.update(interactableHands);

            selfCollisionModel.update();
            environmentCollisionModel.update();

            for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
               robotPartInteractable.update();
         }
      }

      boolean allAreDeleted = true;
      if (interactablesAvailable)
      {
         allAreDeleted &= interactablePelvis.isDeleted();
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
         locomotionManager.calculateWalkPathControlRing3DViewPick(input);

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
         locomotionManager.processWalkPathControlRing3dViewInput(input);

         if (interactablesAvailable)
         {
            environmentCollisionModel.process3DViewInput(input);

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
      robotLowLevelMessenger.renderImGuiWidgets();

      pelvisHeightSlider.renderImGuiWidgets();
      chestPitchSlider.renderImGuiWidgets();
      chestYawSlider.renderImGuiWidgets();

      trajectoryTimeSlider.renderImGuiWidget();

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
            locomotionManager.deleteAll();

            for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
               robotPartInteractable.delete();
         }
      }

      ImGui.separator();
      locomotionManager.renderImGuiWidgets();
      ImGui.separator();

      if (robotHasArms)
         handManager.renderImGuiWidgets();

      if (ImGui.button(labels.get("Set Desired To Current")))
      {
         if (armManager != null)
            armManager.setDesiredToCurrent();
         desiredRobot.setDesiredToCurrent();
      }

      if (interactablesAvailable)
      {
         if (armManager != null)
            armManager.renderImGuiWidgets();

         ImGui.text("Pelvis:");
         ImGuiTools.previousWidgetTooltip("Send with: Spacebar");
         ImGui.sameLine();
         interactablePelvis.renderImGuiWidgets();

         boolean handInteractablesAreDeleted = true;
         if (robotHasArms)
         {
            for (RobotSide side : interactableHands.sides())
            {
               ImGui.text(side.getPascalCaseName() + " hand:");
               ImGui.sameLine();
               interactableHands.get(side).renderImGuiWidgets();
               handInteractablesAreDeleted &= interactableHands.get(side).isDeleted();
            }
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
               locomotionManager.setLegControlModeToSingleSupportFootPosing();
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
               if (showSelfCollisionMeshes.get())
                  selfCollisionModel.getRenderables(renderables, pool);
               if (showEnvironmentCollisionMeshes.get())
                  environmentCollisionModel.getRenderables(renderables, pool);

               for (RDXInteractableRobotLink robotPartInteractable : allInteractableRobotLinks)
                  robotPartInteractable.getVirtualRenderables(renderables, pool);
            }

            locomotionManager.getWalkPathControlRingVirtualRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      desiredRobot.destroy();
      locomotionManager.destroy();
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

   public RDXLocomotionParameters getLocomotionParameters()
   {
      return locomotionManager.getLocomotionParameters();
   }
}
