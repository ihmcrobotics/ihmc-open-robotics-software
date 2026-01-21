package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.ros2log.ROS2LogReplay;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.graphics.RDXRobotPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.hands.RDXHandManager;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationManager;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRManager;
import us.ihmc.rdx.vr.RDXVRTracker;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;
/**
 * Manages the various VR modes and associated state for immersive humanoid control in the RDX UI.
 * <p>
 * Responsible for switching between modes such as whole-body IK streaming, footstep placement,
 * joystick-based walking, and footstep streaming; maintaining their corresponding input logic,
 * GUIs, and visualizations in the VR context.
 * </p>
 * <p>
 * Handles creation and update of VR panels, occlusion and perception visualizers, hand managers, and robot
 * streaming helpers; integrates robot-side per-arm and hand tracking, stereo panel occlusion for rendering,
 * and dynamic control panel presentation.
 * </p>
 */
public class RDXVRModeManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private RDXVRManager vrManager;
   private RDXVRMode mode = RDXVRMode.INPUTS_DISABLED;

   private ImBoolean interactablesEnabled;

   private RDXVRFootstepPlacement footstepPlacer;
   private RDXHandManager handManager;

   @Nullable
   private RDXVRWholeBodyKinematicStreaming kinematicsStreaming;
   private RDXVRFootstepStreaming footstepStreaming;
   private RDXJoystickBasedStepping joystickBasedStepping;

   private RDXVRModeControls vrModeControls;
   private RDX3DSituatedImGuiPanel vrModeControls3DPanel;
   private final FramePose3D vrModeControls3DPanelPose = new FramePose3D();

   private RDXRobotPerceptionVisualizersPanel perceptionVisualizers;
   private ROS2SyncedRobotModel syncedRobot;
   private RDXStereoImagePanel stereoPanel;
   private final Throttler panelOcclusionRateLimiter = new Throttler();
   private final SideDependentList<List<RigidBodyBasics>> syncedRobotArmRigidBodies = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
   private final SideDependentList<List<RigidBodyBasics>> ghostIKRobotArmRigidBodies = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
   private final SideDependentList<Point3D> tempHandPoints = new SideDependentList<>(new Point3D(), new Point3D());

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXRobotPerceptionVisualizersPanel perceptionVisualizers,
                      ROS2ControllerHelper controllerHelper,
                      RetargetingParameters retargetingParameters,
                      boolean createKinematicsStreamingToolboxModule,
                      KinematicsStreamingToolboxParameters kstParameters)
   {
      create(baseUI, syncedRobot, perceptionVisualizers, controllerHelper, retargetingParameters, createKinematicsStreamingToolboxModule, kstParameters, false, null, null, null, false);
   }

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXRobotPerceptionVisualizersPanel perceptionVisualizers,
                      ROS2ControllerHelper controllerHelper,
                      RetargetingParameters retargetingParameters,
                      boolean createKinematicsStreamingToolboxModule,
                      KinematicsStreamingToolboxParameters kstParameters,
                      boolean recordKSTOutput,
                      FullHumanoidRobotModel miniGhostFullRobotModel,
                      RobotDefinition miniGhostRobotDefinition,
                      ROS2LogReplay replayer,
                      boolean enableHeadsetlessTestMode)
   {
      vrManager = baseUI.getVRManager();
      vrManager.setHeadsetlessTestMode(enableHeadsetlessTestMode);
      this.perceptionVisualizers = perceptionVisualizers;
      this.syncedRobot = syncedRobot;

      Collection<RDXPanel> baseUIPanels =  RDXBaseUI.getInstance().getImGuiPanelManager().getPanels();
      for (RDXPanel panel : baseUIPanels)
      {
         if (panel instanceof RDXTeleoperationManager teleoperationPanel)
         {
            interactablesEnabled = teleoperationPanel.getInteractablesEnabled();
            handManager = teleoperationPanel.getArmManager().getHandManager();
            break;
         }
      }
      footstepPlacer = new RDXVRFootstepPlacement(baseUI.getVRManager().getContext(), syncedRobot, controllerHelper);

      if (syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.LEFT) ||
          syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.RIGHT))
      {
         kinematicsStreaming = new RDXVRWholeBodyKinematicStreaming(syncedRobot,
                                                                    controllerHelper,
                                                                    perceptionVisualizers.getRobotVisualizer(),
                                                                    vrManager.getContext(),
                                                                    retargetingParameters,
                                                                    kstParameters,
                                                                    createKinematicsStreamingToolboxModule,
                                                                    recordKSTOutput,
                                                                    handManager,
                                                                    miniGhostFullRobotModel,
                                                                    miniGhostRobotDefinition,
                                                                    replayer);
      }

      joystickBasedStepping = new RDXJoystickBasedStepping(syncedRobot.getRobotModel());
      joystickBasedStepping.create(baseUI, controllerHelper, syncedRobot);

      footstepStreaming = new RDXVRFootstepStreaming(syncedRobot,
                                                     controllerHelper,
                                                     vrManager.getContext(),
                                                     retargetingParameters,
                                                     footstepPlacer);

      vrModeControls = new RDXVRModeControls(this);
      // Panel in VR
      vrModeControls3DPanel = new RDX3DSituatedImGuiPanel("VR Mode Manager", vrModeControls::render);
      vrModeControls3DPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
      vrModeControls3DPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));

      RDXBaseUI.getInstance().getKeyBindings().register("Adjust camera Z height", "Right touchpad scroll");
      RDXBaseUI.getInstance().getKeyBindings().register("Teleport to projected location", "Right B button");
      RDXBaseUI.getInstance().getKeyBindings().register("Teleport to robot", "Right joystick click");
      RDXBaseUI.getInstance().getKeyBindings().register("Toggle left hand panel", "Left joystick click");
      RDXBaseUI.getInstance().getKeyBindings().register("Move 3D panels", "Right trigger click & drag");

      baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
      vrManager.getContext().addVRPickCalculator(this::calculateVRPick);
      vrManager.getContext().addVRInputProcessor(this::processVRInput);

      stereoPanel = new RDXStereoImagePanel(baseUI.getVRManager().getContext(), vrModeControls);
      baseUI.getPrimaryScene().addRenderableProvider(stereoPanel::getRenderables);

      for (RobotSide side : RobotSide.values)
      {
         ArmJointName[] armJointNames = syncedRobot.getRobotModel().getJointMap().getArmJointNames(side);
         for (ArmJointName jointName : armJointNames)
         {
            OneDoFJointBasics syncedRobotArmJoint = syncedRobot.getFullRobotModel().getArmJoint(side, jointName);
            syncedRobotArmRigidBodies.get(side).add(syncedRobotArmJoint.getSuccessor());
            if (kinematicsStreaming != null)
            {
               OneDoFJointBasics ghostIKRobotArmJoint = kinematicsStreaming.getGhostFullRobotModel().getArmJoint(side, jointName);
               ghostIKRobotArmRigidBodies.get(side).add(ghostIKRobotArmJoint.getSuccessor());
            }
         }
      }
   }

   private void calculateVRPick(RDXVRContext vrContext)
   {
      if (vrModeControls.getRenderOnLeftHand().get())
      {
         vrModeControls3DPanel.calculateVRPick(vrContext);
      }
   }

   private void processVRInput(RDXVRContext vrContext)
   {
      if (vrModeControls.getRenderOnLeftHand().get())
      {
         vrModeControls3DPanel.processVRInput(vrContext);
      }

      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
      {
         InputDigitalActionData leftJoystickButton = controller.getJoystickPressActionData();
         if (leftJoystickButton.bChanged() && !leftJoystickButton.bState())
         {
            vrModeControls.getRenderOnLeftHand().set(!vrModeControls.getRenderOnLeftHand().get());
         }

         if (vrModeControls.getRenderOnLeftHand().get())
         {
            vrModeControls3DPanelPose.setToZero(controller.getXForwardZUpControllerFrame());
            vrModeControls3DPanelPose.getOrientation().setYawPitchRoll(Math.PI / 2.0, 0.0, Math.PI / 4.0);
            vrModeControls3DPanelPose.getPosition().addY(-0.05);
            vrModeControls3DPanelPose.changeFrame(ReferenceFrame.getWorldFrame());
            vrModeControls3DPanel.updateDesiredPose(vrModeControls3DPanelPose::get);
         }
      });

      switch (mode)
      {
         case WHOLE_BODY_IK_STREAMING ->
         {
            if (kinematicsStreaming != null)
               kinematicsStreaming.processVRInput();
         }
         case FOOTSTEP_PLACEMENT -> footstepPlacer.processVRInput();
         case FOOTSTEP_STREAMING -> footstepStreaming.processVRInput();
      }
   }

   public void update()
   {
      vrManager.getTeleporter().setBButtonEnabled(mode != RDXVRMode.WHOLE_BODY_IK_STREAMING);
      interactablesEnabled.set(mode == RDXVRMode.INPUTS_DISABLED);

      float opacity = 1.0f;

      if (mode == RDXVRMode.WHOLE_BODY_IK_STREAMING && kinematicsStreaming != null)
         if (!kinematicsStreaming.getShowGhosts() && kinematicsStreaming.getStreamToController())
            opacity = 0.0f;

      for (RobotSide side : vrManager.getContext().getControllers().sides())
         vrManager.getContext().getController(side).setOpacity(opacity);
      for (RDXVRTracker tracker : vrManager.getContext().getTrackers().values())
         tracker.setOpacity(opacity);

      switch (mode)
      {
         case WHOLE_BODY_IK_STREAMING ->
         {
            if (kinematicsStreaming != null)
               kinematicsStreaming.update();
         }
         case JOYSTICK_WALKING -> joystickBasedStepping.update();
         case FOOTSTEP_STREAMING -> footstepStreaming.update();
      }

      if (vrModeControls.getRenderOnLeftHand().get())
         vrModeControls3DPanel.update();
      vrModeControls.update();

      if (perceptionVisualizers.getExperimentalCameraLeftColorImageVisualizer() != null && perceptionVisualizers.getExperimentalCameraRightColorImageVisualizer() != null)
      {
         stereoPanel.update(perceptionVisualizers.getExperimentalCameraLeftColorImageVisualizer().getTexture(),
                            perceptionVisualizers.getExperimentalCameraRightColorImageVisualizer().getTexture(),
                            syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.LEFT),
                            syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.RIGHT),
                            perceptionVisualizers.getZEDModelData().getVerticalFOV());

         if (panelOcclusionRateLimiter.run(UnitConversions.hertzToSeconds(10.0)))
         {
            for (RobotSide side : RobotSide.values)
            {
               checkStereoPanelOcclusions(syncedRobotArmRigidBodies.get(side), perceptionVisualizers.getRobotVisualizer().getMultiBodyGraphic());
               Point3D tempPoint = tempHandPoints.get(side);
               tempPoint.set(syncedRobot.getFullRobotModel().getHandControlFrame(side).getTransformToRoot().getTranslation());
               checkStereoPanelOcclusionWithHandControlFrames(tempPoint, syncedRobotArmRigidBodies.get(side), perceptionVisualizers.getRobotVisualizer().getMultiBodyGraphic());
               if (mode == RDXVRMode.WHOLE_BODY_IK_STREAMING && kinematicsStreaming != null)
               {
                  checkStereoPanelOcclusions(ghostIKRobotArmRigidBodies.get(side), kinematicsStreaming.getGhostRobotGraphic());
                  tempPoint.set(kinematicsStreaming.getGhostFullRobotModel().getHandControlFrame(side).getTransformToRoot().getTranslation());
                  checkStereoPanelOcclusionWithHandControlFrames(tempPoint, ghostIKRobotArmRigidBodies.get(side), kinematicsStreaming.getGhostRobotGraphic());
               }
            }
         }
      }
   }

   /**
    * Checks if the specified hand control frame point is occluded by the stereo panel's view.
    * <p>
    * If it is occluded, hides the last rigid body in the provided list, which is closest to the hand control frame.
    * If it is not occluded, ensures that rigid body is visible.
    * </p>
    *
    * @param handPoint     the hand control frame point to check for occlusion
    * @param rigidBodyList the ordered list of rigid bodies for the arm; last entry is closest to hand
    * @param robotGraphics the graphics object whose visibility set should be updated
    */
   private void checkStereoPanelOcclusionWithHandControlFrames(Point3DReadOnly handPoint, List<RigidBodyBasics> rigidBodyList, RDXMultiBodyGraphic robotGraphics)
   {
      if (stereoPanel.isOccludingView(handPoint))
      {
         robotGraphics.getMultiBody().getRigidBodiesToHide().add(rigidBodyList.get(rigidBodyList.size() - 1).getName());
      }
      else
      {
         robotGraphics.getMultiBody().getRigidBodiesToHide().remove(rigidBodyList.get(rigidBodyList.size() - 1).getName());
      }
   }

   /**
    * Checks occlusion of the origins of a list of robot rigid bodies against the stereo panel's view.
    * <p>
    * For each {@link RigidBodyBasics} in the list, this method tests if its origin (expressed as a {@link Point3D})
    * is currently occluded by the stereo panel. If occluded, the rigid body's name is added to the
    * set of hidden bodies in the provided {@code robotGraphics}. If not occluded, its name is removed from the set.
    * </p>
    *
    * @param rigidBodyList the list of rigid bodies to test for occlusion by the stereo panel
    * @param robotGraphics the graphics instance whose hidden body set is updated for visualization
    */
   private void checkStereoPanelOcclusions(List<RigidBodyBasics> rigidBodyList, RDXMultiBodyGraphic robotGraphics)
   {
      for (RigidBodyBasics rigidBody : rigidBodyList)
      {
         Point3D rigidBodyPosition = new Point3D(rigidBody.getBodyFixedFrame().getTransformToRoot().getTranslation());
         if (stereoPanel.isOccludingView(rigidBodyPosition))
         {
            robotGraphics.getMultiBody().getRigidBodiesToHide().add(rigidBody.getName());
         }
         else
         {
            robotGraphics.getMultiBody().getRigidBodiesToHide().remove(rigidBody.getName());
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.radioButton(labels.get(RDXVRMode.INPUTS_DISABLED.getReadableName()), mode == RDXVRMode.INPUTS_DISABLED))
      {
         mode = RDXVRMode.INPUTS_DISABLED;
         disableKinematicsStreaming();
         footstepPlacer.reset();
         footstepStreaming.reset();
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.FOOTSTEP_PLACEMENT.getReadableName()), mode == RDXVRMode.FOOTSTEP_PLACEMENT))
      {
         mode = RDXVRMode.FOOTSTEP_PLACEMENT;
         disableKinematicsStreaming();
         interactablesEnabled.set(false);
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.FOOTSTEP_STREAMING.getReadableName()), mode == RDXVRMode.FOOTSTEP_STREAMING))
      {
         mode = RDXVRMode.FOOTSTEP_STREAMING;
         disableKinematicsStreaming();
         footstepStreaming.reset();
         interactablesEnabled.set(false);
      }
      if (kinematicsStreaming == null)
      {
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.WHOLE_BODY_IK_STREAMING.getReadableName()), mode == RDXVRMode.WHOLE_BODY_IK_STREAMING))
      {
         mode = RDXVRMode.WHOLE_BODY_IK_STREAMING;
         footstepPlacer.reset();
         footstepStreaming.reset();
         interactablesEnabled.set(false);
      }
      if (kinematicsStreaming == null)
      {
         ImGui.popStyleColor();
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.JOYSTICK_WALKING.getReadableName()), mode == RDXVRMode.JOYSTICK_WALKING))
      {
         mode = RDXVRMode.JOYSTICK_WALKING;
         disableKinematicsStreaming();
         footstepPlacer.reset();
         interactablesEnabled.set(false);
      }
   }

   private void disableKinematicsStreaming()
   {
      if (kinematicsStreaming != null)
         kinematicsStreaming.setKSTEnabled(false);
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         switch (mode)
         {
            case FOOTSTEP_PLACEMENT, FOOTSTEP_STREAMING -> footstepPlacer.getRenderables(renderables, pool);
            case WHOLE_BODY_IK_STREAMING ->
            {
               if (kinematicsStreaming != null)
                  kinematicsStreaming.getVirtualRenderables(renderables, pool, sceneLevels);
            }
            case JOYSTICK_WALKING -> joystickBasedStepping.getRenderables(renderables, pool);
         }

         if (vrModeControls.getRenderOnLeftHand().get())
            vrModeControls3DPanel.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      vrModeControls3DPanel.dispose();
      if (kinematicsStreaming != null)
         kinematicsStreaming.destroy();
      joystickBasedStepping.destroy();
      footstepPlacer.destroy();
      footstepStreaming.destroy();
      stereoPanel.destroy();
   }

   public void setMode(RDXVRMode mode)
   {
      this.mode = mode;
   }

   public RDXVRMode getMode()
   {
      return mode;
   }

   public RDXVRFootstepPlacement getFootstepPlacer()
   {
      return footstepPlacer;
   }

   @Nullable
   public RDXVRWholeBodyKinematicStreaming getKinematicsStreaming()
   {
      return kinematicsStreaming;
   }

   public RDXJoystickBasedStepping getJoystickBasedStepping()
   {
      return joystickBasedStepping;
   }

   public RDXVRModeControls getControls()
   {
      return vrModeControls;
   }
}