package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;
import us.ihmc.rdx.ui.processes.RestartableJavaProcess;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Set;

/**
 * TODO: Figure out how to incorporate this class with things better.
 */
public class RDXVRModeManager
{
   private ROS2SyncedRobotModel syncedRobot;
   private RDXVRLaserFootstepMode laserFootstepMode;
   private RDXVRHandPlacedFootstepMode handPlacedFootstepMode;
   private RDXVRKinematicsStreamingMode kinematicsStreamingMode;
   private RDXJoystickBasedStepping joystickBasedStepping;
   private RDX3DSituatedImGuiPanel leftHandPanel;
   private final FramePose3D leftHandPanelPose = new FramePose3D();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXVRMode mode = RDXVRMode.INPUTS_DISABLED;
   private RDXPanelPlacementMode panelPlacementMode = RDXPanelPlacementMode.MANUAL_PLACEMENT;
   private boolean renderPanel;
   private final ImBoolean showFloatingVideoPanel = new ImBoolean(false);
   private final Notification showFloatVideoPanelNotification = new Notification();

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper controllerHelper,
                      RestartableJavaProcess kinematicsStreamingToolboxProcess)
   {
      this.syncedRobot = syncedRobot;
      handPlacedFootstepMode = new RDXVRHandPlacedFootstepMode();
      handPlacedFootstepMode.create(syncedRobot.getRobotModel(), controllerHelper);
      laserFootstepMode = new RDXVRLaserFootstepMode();

      if (syncedRobot.getRobotModel().getRobotVersion().hasArms())
      {
         kinematicsStreamingMode = new RDXVRKinematicsStreamingMode(syncedRobot.getRobotModel(), controllerHelper, kinematicsStreamingToolboxProcess);
         kinematicsStreamingMode.create(baseUI.getVRManager().getContext());
      }

      joystickBasedStepping = new RDXJoystickBasedStepping(syncedRobot.getRobotModel());
      joystickBasedStepping.create(baseUI, controllerHelper, syncedRobot);

      baseUI.getImGuiPanelManager().addPanel("VR Mode Manager", this::renderImGuiWidgets);

      leftHandPanel = new RDX3DSituatedImGuiPanel("VR Mode Manager", this::renderImGuiWidgets);
      leftHandPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
      leftHandPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
      baseUI.getVRManager().getContext().addVRPickCalculator(leftHandPanel::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(leftHandPanel::processVRInput);
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      renderPanel = vrContext.getHeadset().isConnected() && vrContext.getController(RobotSide.LEFT).isConnected();

      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
      {
         leftHandPanelPose.setToZero(controller.getXForwardZUpControllerFrame());
         leftHandPanelPose.getOrientation().setYawPitchRoll(Math.PI / 2.0, 0.0, Math.PI / 4.0);
         leftHandPanelPose.getPosition().addY(-0.05);
         leftHandPanelPose.changeFrame(ReferenceFrame.getWorldFrame());
         leftHandPanel.updateDesiredPose(leftHandPanelPose::get);
      });

      switch (mode)
      {
         case FOOTSTEP_PLACEMENT -> handPlacedFootstepMode.processVRInput(vrContext);
         case LASER_PLACEMENT -> laserFootstepMode.processVRInput(vrContext);
         case WHOLE_BODY_IK_STREAMING ->
         {
            if (kinematicsStreamingMode != null)
               kinematicsStreamingMode.processVRInput(vrContext);
         }
      }
   }

   public void update()
   {
      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.update(mode == RDXVRMode.WHOLE_BODY_IK_STREAMING);
      leftHandPanel.update();
      joystickBasedStepping.update(mode == RDXVRMode.JOYSTICK_WALKING);
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Teleport: Right B button");
      ImGui.text("Adjust user Z height: Right touchpad up/down");
      ImGui.text("ImGui panels: Point and use right trigger to click and drag");
      if (ImGui.checkbox(labels.get("Floating video panel"), showFloatingVideoPanel))
      {
         if (showFloatingVideoPanel.get())
            showFloatVideoPanelNotification.set();
      }
      if (showFloatingVideoPanel.get())
      {
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Manually place"), panelPlacementMode == RDXPanelPlacementMode.MANUAL_PLACEMENT))
         {
            panelPlacementMode = RDXPanelPlacementMode.MANUAL_PLACEMENT;
         }
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Follow headset"), panelPlacementMode == RDXPanelPlacementMode.FOLLOW_HEADSET))
         {
            panelPlacementMode = RDXPanelPlacementMode.FOLLOW_HEADSET;
         }
      }

      if (ImGui.radioButton(labels.get("Inputs disabled"), mode == RDXVRMode.INPUTS_DISABLED))
      {
         mode = RDXVRMode.INPUTS_DISABLED;
      }
      if (ImGui.radioButton(labels.get("Footstep placement"), mode == RDXVRMode.FOOTSTEP_PLACEMENT))
      {
         mode = RDXVRMode.FOOTSTEP_PLACEMENT;
      }
      if (ImGui.radioButton(labels.get("Laser placement"), mode == RDXVRMode.LASER_PLACEMENT))
      {
         mode = RDXVRMode.LASER_PLACEMENT;
      }
      if (ImGui.radioButton(labels.get("Whole body IK streaming"), mode == RDXVRMode.WHOLE_BODY_IK_STREAMING))
      {
         mode = RDXVRMode.WHOLE_BODY_IK_STREAMING;
         //         if (!kinematicsStreamingMode.getKinematicsStreamingToolboxProcess().isStarted())
         //            kinematicsStreamingMode.getKinematicsStreamingToolboxProcess().start();
      }
      if (ImGui.radioButton(labels.get("Joystick walking"), mode == RDXVRMode.JOYSTICK_WALKING))
      {
         mode = RDXVRMode.JOYSTICK_WALKING;
      }

      switch (mode)
      {
         case INPUTS_DISABLED ->
         {
            ImGui.text("Press right joystick button to teleport the playspace to the robot's location.");
         }
         case FOOTSTEP_PLACEMENT ->
         {
            handPlacedFootstepMode.renderImGuiWidgets();
         }
         case LASER_PLACEMENT ->
         {
            laserFootstepMode.renderImGuiWidgets();
         }
         case WHOLE_BODY_IK_STREAMING ->
         {
            if (kinematicsStreamingMode != null)
               kinematicsStreamingMode.renderImGuiWidgets();
         }
         case JOYSTICK_WALKING ->
         {
            joystickBasedStepping.renderImGuiWidgets();
         }
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         switch (mode)
         {
            case FOOTSTEP_PLACEMENT ->
            {
               handPlacedFootstepMode.getRenderables(renderables, pool);
            }
            case LASER_PLACEMENT ->
            {
               laserFootstepMode.getRenderables(renderables, pool);
            }
            case WHOLE_BODY_IK_STREAMING ->
            {
               if (kinematicsStreamingMode != null)
                  kinematicsStreamingMode.getVirtualRenderables(renderables, pool, sceneLevels);
            }
            case JOYSTICK_WALKING ->
            {
               joystickBasedStepping.getRenderables(renderables, pool);
            }
         }

         if (renderPanel)
         {
            leftHandPanel.getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      leftHandPanel.dispose();
      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.destroy();
      joystickBasedStepping.destroy();
   }

   public RDXVRKinematicsStreamingMode getKinematicsStreamingMode()
   {
      return kinematicsStreamingMode;
   }

   public ImBoolean getShowFloatingVideoPanel()
   {
      return showFloatingVideoPanel;
   }

   public Notification getShowFloatVideoPanelNotification()
   {
      return showFloatVideoPanelNotification;
   }

   public RDXPanelPlacementMode getVideoPanelPlacementMode()
   {
      return panelPlacementMode;
   }

   public RDXVRHandPlacedFootstepMode getHandPlacedFootstepMode()
   {
      return handPlacedFootstepMode;
   }

   public RDXVRLaserFootstepMode getLaserFootstepMode()
   {
      return laserFootstepMode;
   }
}
