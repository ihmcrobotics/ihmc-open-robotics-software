package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.motionRetargeting.DefaultRetargetingParameters;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;
import us.ihmc.rdx.ui.affordances.RDXManualFootstepPlacement;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.teleoperation.RDXHandConfigurationManager;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationManager;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;
import java.util.Collection;
import java.util.Set;

public class RDXVRModeManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private RDXVRMode mode = RDXVRMode.INPUTS_DISABLED;
   private RDXVRHandPlacedFootstepMode handPlacedFootstepMode;
   @Nullable
   private RDXVRKinematicsStreamingMode kinematicsStreamingMode;
   private RDXJoystickBasedStepping joystickBasedStepping;
   private RDXVRStereoVision stereoVision;
   private RDXVRModeControls vrModeControls;

   private RDX3DSituatedImGuiPanel vrModeControls3DPanel;
   private final FramePose3D vrModeControls3DPanelPose = new FramePose3D();
   private RDXROS2RobotVisualizer robotVisualizer;
   private ImBoolean interactablesEnabled;
   private ControllerStatusTracker controllerStatusTracker;
   private RDXManualFootstepPlacement footstepPlacer;
   private RDXHandConfigurationManager handManager;

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXROS2RobotVisualizer robotVisualizer,
                      ROS2ControllerHelper controllerHelper,
                      boolean createKinematicsStreamingToolboxModule)
   {
      create(baseUI,
             syncedRobot,
             robotVisualizer,
             controllerHelper,
             new DefaultRetargetingParameters(),
             new SceneGraph(),
             createKinematicsStreamingToolboxModule);
   }

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXROS2RobotVisualizer robotVisualizer,
                      ROS2ControllerHelper controllerHelper,
                      RetargetingParameters retargetingParameters,
                      boolean createKinematicsStreamingToolboxModule)
   {
      create(baseUI, syncedRobot, robotVisualizer, controllerHelper, retargetingParameters, new SceneGraph(), createKinematicsStreamingToolboxModule);
   }

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXROS2RobotVisualizer robotVisualizer,
                      ROS2ControllerHelper controllerHelper,
                      RetargetingParameters retargetingParameters,
                      SceneGraph sceneGraph,
                      boolean createKinematicsStreamingToolboxModule)
   {
      this.robotVisualizer = robotVisualizer;

      Collection<RDXPanel> baseUIPanels =  RDXBaseUI.getInstance().getImGuiPanelManager().getPanels();
      for (RDXPanel panel : baseUIPanels)
      {
         if (panel instanceof RDXTeleoperationManager teleoperationPanel)
         {
            interactablesEnabled = teleoperationPanel.getInteractablesEnabled();
            controllerStatusTracker = teleoperationPanel.getControllerStatusTracker();
            footstepPlacer = teleoperationPanel.getLocomotionManager().getManualFootstepPlacement();
            handManager = teleoperationPanel.getArmManager().getHandManager();
            break;
         }
      }

      handPlacedFootstepMode = new RDXVRHandPlacedFootstepMode(baseUI.getVRManager().getContext());
      handPlacedFootstepMode.create(syncedRobot, controllerHelper);

      if (syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.LEFT) || syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.RIGHT))
      {
         kinematicsStreamingMode = new RDXVRKinematicsStreamingMode(syncedRobot,
                                                                    controllerHelper,
                                                                    baseUI.getVRManager().getContext(),
                                                                    retargetingParameters,
                                                                    sceneGraph,
                                                                    controllerStatusTracker,
                                                                    footstepPlacer,
                                                                    handManager);
         kinematicsStreamingMode.create(createKinematicsStreamingToolboxModule);
      }

      joystickBasedStepping = new RDXJoystickBasedStepping(syncedRobot.getRobotModel());
      joystickBasedStepping.create(baseUI, controllerHelper, syncedRobot);

      stereoVision = new RDXVRStereoVision(syncedRobot.getReferenceFrames());
      vrModeControls = new RDXVRModeControls(this);

      // Panel in VR
      vrModeControls3DPanel = new RDX3DSituatedImGuiPanel("VR Mode Manager", vrModeControls::render);
      vrModeControls3DPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
      vrModeControls3DPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
      baseUI.getVRManager().getContext().addVRPickCalculator(vrModeControls3DPanel::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(vrModeControls3DPanel::processVRInput);

      RDXBaseUI.getInstance().getKeyBindings().register("Teleport", "Right B button");
      RDXBaseUI.getInstance().getKeyBindings().register("Adjust camera Z height", "Right touchpad scroll");
      RDXBaseUI.getInstance().getKeyBindings().register("Move 3D panels", "Right trigger click & drag");
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      if (vrModeControls.getRenderOnLeftHand().get())
      {
         vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
         {
            vrModeControls3DPanelPose.setToZero(controller.getXForwardZUpControllerFrame());
            vrModeControls3DPanelPose.getOrientation().setYawPitchRoll(Math.PI / 2.0, 0.0, Math.PI / 4.0);
            vrModeControls3DPanelPose.getPosition().addY(-0.05);
            vrModeControls3DPanelPose.changeFrame(ReferenceFrame.getWorldFrame());
            vrModeControls3DPanel.updateDesiredPose(vrModeControls3DPanelPose::get);
         });
      }

      switch (mode)
      {
         case FOOTSTEP_PLACEMENT -> handPlacedFootstepMode.processVRInput();
         case WHOLE_BODY_IK_STREAMING ->
         {
            if (kinematicsStreamingMode != null)
               kinematicsStreamingMode.processVRInput();
         }
      }
   }

   public void update()
   {
      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.update(mode == RDXVRMode.WHOLE_BODY_IK_STREAMING);
      if (vrModeControls.getRenderOnLeftHand().get())
         vrModeControls3DPanel.update();
      joystickBasedStepping.update(mode == RDXVRMode.JOYSTICK_WALKING);
      vrModeControls.update();

      if (mode == RDXVRMode.FOOTSTEP_PLACEMENT)
      { // Disable interactables because we do not want to collide with the walking control ring
         interactablesEnabled.set(false);
      }
      else if (!interactablesEnabled.get())
      {
         interactablesEnabled.set(true);
      }

      // Fade robot graphics
      if (kinematicsStreamingMode.isStreaming() && stereoVision.isEnabled())
      { // Fade graphics out if in stereo vision mode while streaming
         kinematicsStreamingMode.visualizeIKPreviewGraphic(false);
         robotVisualizer.fadeVisuals(0.0f, 0.01f);
      }
      else if (mode == RDXVRMode.WHOLE_BODY_IK_STREAMING && (stereoVision.getDisabledNotification().poll()
                                                             || kinematicsStreamingMode.getStreamingDisabledNotification().poll()
                                                             || robotVisualizer.isFading()))
      { // Fade graphics in if either stereo vision or streaming are exited
         robotVisualizer.fadeVisuals(1.0f, 0.05f);
         if (!robotVisualizer.isFading())
            kinematicsStreamingMode.visualizeIKPreviewGraphic(true);
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.radioButton(labels.get(RDXVRMode.INPUTS_DISABLED.getReadableName()), mode == RDXVRMode.INPUTS_DISABLED))
      {
         mode = RDXVRMode.INPUTS_DISABLED;
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.FOOTSTEP_PLACEMENT.getReadableName()), mode == RDXVRMode.FOOTSTEP_PLACEMENT))
      {
         mode = RDXVRMode.FOOTSTEP_PLACEMENT;
      }
      if (kinematicsStreamingMode == null)
      {
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.WHOLE_BODY_IK_STREAMING.getReadableName()), mode == RDXVRMode.WHOLE_BODY_IK_STREAMING))
      {
         mode = RDXVRMode.WHOLE_BODY_IK_STREAMING;
      }
      if (kinematicsStreamingMode == null)
      {
         ImGui.popStyleColor();
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.JOYSTICK_WALKING.getReadableName()), mode == RDXVRMode.JOYSTICK_WALKING))
      {
         mode = RDXVRMode.JOYSTICK_WALKING;
      }
   }

   public void render()
   {
      stereoVision.renderProjection();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         switch (mode)
         {
            case FOOTSTEP_PLACEMENT -> handPlacedFootstepMode.getRenderables(renderables, pool);
            case WHOLE_BODY_IK_STREAMING ->
            {
               if (kinematicsStreamingMode != null)
                  kinematicsStreamingMode.getVirtualRenderables(renderables, pool, sceneLevels);
            }
            case JOYSTICK_WALKING -> joystickBasedStepping.getRenderables(renderables, pool);
         }
      }

      if (stereoVision.isEnabled())
         stereoVision.getDualBlackflySphericalProjection().getRenderables(renderables, pool, sceneLevels);

      if (vrModeControls.getRenderOnLeftHand().get())
         vrModeControls3DPanel.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      vrModeControls3DPanel.dispose();
      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.destroy();
      joystickBasedStepping.destroy();
      stereoVision.getDualBlackflySphericalProjection().shutdown();
   }

   public RDXVRMode getMode()
   {
      return mode;
   }

   public RDXVRHandPlacedFootstepMode getHandPlacedFootstepMode()
   {
      return handPlacedFootstepMode;
   }

   @Nullable
   public RDXVRKinematicsStreamingMode getKinematicsStreamingMode()
   {
      return kinematicsStreamingMode;
   }

   public RDXJoystickBasedStepping getJoystickBasedStepping()
   {
      return joystickBasedStepping;
   }

   public RDXVRStereoVision getStereoVision()
   {
      return stereoVision;
   }
}