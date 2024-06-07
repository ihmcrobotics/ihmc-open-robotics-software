package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.motionRetargeting.DefaultRetargetingParameters;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.RDXScriptedTrajectoryStreamer.ScriptedTrajectoryType;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Set;

/**
 * TODO: Figure out how to incorporate this class with things better.
 */
public class RDXVRModeManager
{
   private RDXVRKinematicsStreamingMode kinematicsStreamingMode;
   private RDXScriptedMotionMode scriptedMotionMode;
//   private RDX3DSituatedImGuiPanel leftHandPanel;
//   private final FramePose3D leftHandPanelPose = new FramePose3D();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXVRMode mode = RDXVRMode.WHOLE_BODY_IK_STREAMING;
   private RDXVRPanelPlacementMode panelPlacementMode = RDXVRPanelPlacementMode.MANUAL_PLACEMENT;
//   private boolean renderPanel;
   private final ImBoolean showFloatingVideoPanel = new ImBoolean(false);
   private final Notification showFloatVideoPanelNotification = new Notification();
   private final ImInt selectedVRMode = new ImInt(RDXVRMode.WHOLE_BODY_IK_STREAMING.ordinal());
   private final String[] vrModeNames = new String[RDXVRMode.values().length];

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper controllerHelper)
   {
      create(baseUI, syncedRobot, controllerHelper, new DefaultRetargetingParameters(), new SceneGraph());
   }

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper controllerHelper,
                      RetargetingParameters retargetingParameters)
   {
      create(baseUI, syncedRobot, controllerHelper, retargetingParameters, new SceneGraph());
   }

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper controllerHelper,
                      RetargetingParameters retargetingParameters,
                      SceneGraph sceneGraph)
   {
      if (syncedRobot.getRobotModel().getRobotVersion().hasBothArms())
      {
         kinematicsStreamingMode = new RDXVRKinematicsStreamingMode(syncedRobot, controllerHelper, retargetingParameters, sceneGraph);
         kinematicsStreamingMode.create(baseUI.getVRManager().getContext());

         scriptedMotionMode = new RDXScriptedMotionMode(syncedRobot, controllerHelper, sceneGraph);
         scriptedMotionMode.create(baseUI.getVRManager().getContext());
      }

      baseUI.getImGuiPanelManager().addPanel("VR Mode Manager", this::renderImGuiWidgets);

//      leftHandPanel = new RDX3DSituatedImGuiPanel("VR Mode Manager", this::renderImGuiWidgets);
//      leftHandPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
//      leftHandPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
//      baseUI.getVRManager().getContext().addVRPickCalculator(leftHandPanel::calculateVRPick);
//      baseUI.getVRManager().getContext().addVRInputProcessor(leftHandPanel::processVRInput);

      for (RDXVRMode mode : RDXVRMode.values())
      {
         vrModeNames[mode.ordinal()] = mode.name();
      }
   }

   public void processVRInput(RDXVRContext vrContext)
   {
//      renderPanel = vrContext.getHeadset().isConnected() && vrContext.getController(RobotSide.LEFT).isConnected();

//      for (RobotSide side : RobotSide.values)
//      {
//         vrContext.getController(side).runIfConnected(controller ->
//         {
//            // During kinematic streaming, the only way to get out of it is the left hand panel.
//            // TODO (AM): Fix the VR panel not closing when in IK streaming mode
//            controller.setExclusiveAccess(mode == RDXVRMode.WHOLE_BODY_IK_STREAMING ? leftHandPanel : null);
//
//            if (side == RobotSide.LEFT)
//            {
//               leftHandPanelPose.setToZero(controller.getXForwardZUpControllerFrame());
//               leftHandPanelPose.getOrientation().setYawPitchRoll(Math.PI / 2.0, 0.0, Math.PI / 4.0);
//               leftHandPanelPose.getPosition().addY(-0.05);
//               leftHandPanelPose.changeFrame(ReferenceFrame.getWorldFrame());
//               leftHandPanel.updateDesiredPose(leftHandPanelPose::get);
//            }
//         });
//      }

      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
                                                             {
                                                                InputDigitalActionData bButton = controller.getBButtonActionData();
                                                                if (bButton.bChanged() && !bButton.bState())
                                                                {
                                                                   //TODO: switch modes here instead of executing scripted
                                                                   if (mode == RDXVRMode.SCRIPTED_MOTION && !scriptedMotionMode.isScriptedMotionExecuting())
                                                                   {
                                                                      mode = RDXVRMode.WHOLE_BODY_IK_STREAMING;
                                                                   }
                                                                   else if (mode == RDXVRMode.WHOLE_BODY_IK_STREAMING)
                                                                   {
                                                                      mode = RDXVRMode.SCRIPTED_MOTION;
                                                                   }
                                                                }
                                                             });

      switch (mode)
      {
         case WHOLE_BODY_IK_STREAMING ->
         {
            if (kinematicsStreamingMode != null)
               kinematicsStreamingMode.processVRInput(vrContext);
         }
         case SCRIPTED_MOTION ->
         {
            if (scriptedMotionMode != null)
               scriptedMotionMode.processVRInput(vrContext);
         }
      }
   }

   public void update()
   {
      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.update(mode == RDXVRMode.WHOLE_BODY_IK_STREAMING);
      if (scriptedMotionMode != null)
         scriptedMotionMode.update(mode == RDXVRMode.SCRIPTED_MOTION);
//      leftHandPanel.update();
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Teleport: Right joystick click");

      if (ImGui.checkbox(labels.get("Floating video panel"), showFloatingVideoPanel))
      {
         if (showFloatingVideoPanel.get())
            showFloatVideoPanelNotification.set();
      }
      if (showFloatingVideoPanel.get())
      {
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Manually place"), panelPlacementMode == RDXVRPanelPlacementMode.MANUAL_PLACEMENT))
         {
            panelPlacementMode = RDXVRPanelPlacementMode.MANUAL_PLACEMENT;
         }
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Follow headset"), panelPlacementMode == RDXVRPanelPlacementMode.FOLLOW_HEADSET))
         {
            panelPlacementMode = RDXVRPanelPlacementMode.FOLLOW_HEADSET;
         }
      }

      ImGui.newLine();
      ImGui.text("VR Mode:");
      ImGui.pushItemWidth(200.0f);
      ImGui.combo(labels.getHidden("VR Mode Combo"), selectedVRMode, vrModeNames);
      // Set the current mode to the value that was selected in the combo box
      mode = RDXVRMode.values()[selectedVRMode.get()];

      switch (mode)
      {
         case INPUTS_DISABLED ->
         {
            ImGui.text("Press right joystick button to teleport the playspace to the robot's location.");
         }
         case WHOLE_BODY_IK_STREAMING ->
         {
            if (kinematicsStreamingMode != null)
               kinematicsStreamingMode.renderImGuiWidgets();
         }
         case SCRIPTED_MOTION ->
         {
            if (scriptedMotionMode != null)
               scriptedMotionMode.renderImGuiWidgets();
         }
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
      {
         switch (mode)
         {
            case WHOLE_BODY_IK_STREAMING ->
            {
               if (kinematicsStreamingMode != null)
                  kinematicsStreamingMode.getVirtualRenderables(renderables, pool, sceneLevels);
            }
         }

//         if (renderPanel)
//         {
//            leftHandPanel.getRenderables(renderables, pool);
//         }
      }
   }

   public void destroy()
   {
//      leftHandPanel.dispose();
      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.destroy();
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

   public RDXVRPanelPlacementMode getVideoPanelPlacementMode()
   {
      return panelPlacementMode;
   }

   public void setVideoPanelPlacementMode(RDXVRPanelPlacementMode panelPlacementMode)
   {
      this.panelPlacementMode = panelPlacementMode;
   }

}