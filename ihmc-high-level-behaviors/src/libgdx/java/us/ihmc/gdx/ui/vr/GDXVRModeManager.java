package us.ihmc.gdx.ui.vr;

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
import us.ihmc.gdx.imgui.GDX3DSituatedImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.GDXJoystickBasedStepping;
import us.ihmc.gdx.ui.missionControl.processes.RestartableJavaProcess;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * TODO: Figure out how to incorporate this class with things better.
 */
public class GDXVRModeManager
{
   private ROS2SyncedRobotModel syncedRobot;
   private GDXVRHandPlacedFootstepMode handPlacedFootstepMode;
   private GDXVRKinematicsStreamingMode kinematicsStreamingMode;
   private GDXJoystickBasedStepping joystickBasedStepping;
   private GDX3DSituatedImGuiPanel leftHandPanel;
   private final FramePose3D leftHandPanelPose = new FramePose3D();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private GDXVRMode mode = GDXVRMode.INPUTS_DISABLED;
   private boolean renderPanel;
   private final ImBoolean showFloatingVideoPanel = new ImBoolean(false);
   private final Notification showFloatVideoPanelNotification = new Notification();

   public void create(GDXImGuiBasedUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper controllerHelper,
                      RestartableJavaProcess kinematicsStreamingToolboxProcess)
   {
      this.syncedRobot = syncedRobot;
      handPlacedFootstepMode = new GDXVRHandPlacedFootstepMode();
      handPlacedFootstepMode.create(syncedRobot.getRobotModel(), controllerHelper);

      kinematicsStreamingMode = new GDXVRKinematicsStreamingMode(syncedRobot.getRobotModel(), controllerHelper, kinematicsStreamingToolboxProcess);
      kinematicsStreamingMode.create(baseUI.getVRManager().getContext());

      joystickBasedStepping = new GDXJoystickBasedStepping(syncedRobot.getRobotModel());
      joystickBasedStepping.create(baseUI, controllerHelper, syncedRobot);

      baseUI.getImGuiPanelManager().addPanel("VR Mode Manager", this::renderImGuiWidgets);

      leftHandPanel = new GDX3DSituatedImGuiPanel("VR Mode Manager", this::renderImGuiWidgets);
      leftHandPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
      leftHandPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
      baseUI.getVRManager().getContext().addVRPickCalculator(leftHandPanel::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(leftHandPanel::processVRInput);
   }

   public void processVRInput(GDXVRContext vrContext)
   {
      renderPanel = vrContext.getHeadset().isConnected() && vrContext.getController(RobotSide.LEFT).isConnected();

      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
         if (mode == GDXVRMode.INPUTS_DISABLED && controller.getAButtonActionData().bChanged() && !controller.getAButtonActionData().bState())
         {
            vrContext.teleport(transform ->
            {
               syncedRobot.getReferenceFrames().getMidFeetUnderPelvisFrame().getTransformToDesiredFrame(transform, ReferenceFrame.getWorldFrame());
            });
         }
      });

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
         case WHOLE_BODY_IK_STREAMING -> kinematicsStreamingMode.processVRInput(vrContext);
      }
   }

   public void update()
   {
      kinematicsStreamingMode.update(mode == GDXVRMode.WHOLE_BODY_IK_STREAMING);
      leftHandPanel.update();
      joystickBasedStepping.update(mode == GDXVRMode.JOYSTICK_WALKING);
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
      if (ImGui.radioButton(labels.get("Inputs disabled"), mode == GDXVRMode.INPUTS_DISABLED))
      {
         mode = GDXVRMode.INPUTS_DISABLED;
      }
      if (ImGui.radioButton(labels.get("Footstep placement"), mode == GDXVRMode.FOOTSTEP_PLACEMENT))
      {
         mode = GDXVRMode.FOOTSTEP_PLACEMENT;
      }
      if (ImGui.radioButton(labels.get("Whole body IK streaming"), mode == GDXVRMode.WHOLE_BODY_IK_STREAMING))
      {
         mode = GDXVRMode.WHOLE_BODY_IK_STREAMING;
//         if (!kinematicsStreamingMode.getKinematicsStreamingToolboxProcess().isStarted())
//            kinematicsStreamingMode.getKinematicsStreamingToolboxProcess().start();
      }
      if (ImGui.radioButton(labels.get("Joystick walking"), mode == GDXVRMode.JOYSTICK_WALKING))
      {
         mode = GDXVRMode.JOYSTICK_WALKING;
      }

      switch (mode)
      {
         case INPUTS_DISABLED ->
         {
            ImGui.text("Press right A button to teleport the playspace to the robot's location.");
         }
         case FOOTSTEP_PLACEMENT ->
         {
            handPlacedFootstepMode.renderImGuiWidgets();
         }
         case WHOLE_BODY_IK_STREAMING ->
         {
            kinematicsStreamingMode.renderImGuiWidgets();
         }
         case JOYSTICK_WALKING ->
         {
            joystickBasedStepping.renderImGuiWidgets();
         }
      }
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      switch (mode)
      {
         case FOOTSTEP_PLACEMENT ->
         {
            handPlacedFootstepMode.getRenderables(renderables, pool);
         }
         case WHOLE_BODY_IK_STREAMING ->
         {
            kinematicsStreamingMode.getVirtualRenderables(renderables, pool);
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

   public void destroy()
   {
      leftHandPanel.dispose();
      kinematicsStreamingMode.destroy();
   }

   public GDXVRKinematicsStreamingMode getKinematicsStreamingMode()
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

   public GDXVRHandPlacedFootstepMode getHandPlacedFootstepMode()
   {
      return handPlacedFootstepMode;
   }
}
