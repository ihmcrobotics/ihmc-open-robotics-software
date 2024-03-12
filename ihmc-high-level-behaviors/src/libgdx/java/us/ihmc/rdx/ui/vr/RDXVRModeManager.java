package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.motionRetargeting.DefaultRetargetingParameters;
import us.ihmc.motionRetargeting.RetargetingParameters;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Set;

/**
 * TODO: Figure out how to incorporate this class with things better.
 */
public class RDXVRModeManager
{
   private RDXVRMode mode = RDXVRMode.INPUTS_DISABLED;
   private RDXVRHandPlacedFootstepMode handPlacedFootstepMode;
   private RDXVRKinematicsStreamingMode kinematicsStreamingMode;
   private RDXJoystickBasedStepping joystickBasedStepping;
   private RDX3DSituatedImGuiPanel leftHandPanel;
   private RDXVRStereoVision stereoVision;
   private RDXVRModeControls vrModeControls;
   private final FramePose3D leftHandPanelPose = new FramePose3D();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public void create(RDXBaseUI baseUI, ROS2SyncedRobotModel syncedRobot, ROS2ControllerHelper controllerHelper)
   {
      create(baseUI, syncedRobot, controllerHelper, new DefaultRetargetingParameters(), new SceneGraph());
   }

   public void create(RDXBaseUI baseUI, ROS2SyncedRobotModel syncedRobot, ROS2ControllerHelper controllerHelper, RetargetingParameters retargetingParameters)
   {
      create(baseUI, syncedRobot, controllerHelper, retargetingParameters, new SceneGraph());
   }

   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper controllerHelper,
                      RetargetingParameters retargetingParameters,
                      SceneGraph sceneGraph)
   {
      handPlacedFootstepMode = new RDXVRHandPlacedFootstepMode();
      handPlacedFootstepMode.create(syncedRobot.getRobotModel(), controllerHelper);

      if (syncedRobot.getRobotModel().getRobotVersion().hasBothArms())
      {
         kinematicsStreamingMode = new RDXVRKinematicsStreamingMode(syncedRobot, controllerHelper, retargetingParameters, sceneGraph);
         kinematicsStreamingMode.create(baseUI.getVRManager().getContext());
      }

      joystickBasedStepping = new RDXJoystickBasedStepping(syncedRobot.getRobotModel());
      joystickBasedStepping.create(baseUI, controllerHelper, syncedRobot);

      // Panel in VR
      leftHandPanel = new RDX3DSituatedImGuiPanel("VR Mode Manager", this::renderImGuiWidgets);
      leftHandPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
      leftHandPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
      baseUI.getVRManager().getContext().addVRPickCalculator(leftHandPanel::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(leftHandPanel::processVRInput);

      stereoVision = new RDXVRStereoVision(syncedRobot.getReferenceFrames().getMidFootZUpGroundFrame(), baseUI.getVRManager().getContext().getHeadset().getXForwardZUpHeadsetFrame());

      vrModeControls = new RDXVRModeControls(baseUI, this);

      RDXBaseUI.getInstance().getKeyBindings().register("Teleport", "Right B button");
      RDXBaseUI.getInstance().getKeyBindings().register("Adjust camera Z height", "Right touchpad scroll");
      RDXBaseUI.getInstance().getKeyBindings().register("Move 3D panels", "Right trigger click & drag");
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            if (side == RobotSide.LEFT)
            {
               leftHandPanelPose.setToZero(controller.getXForwardZUpControllerFrame());
               leftHandPanelPose.getOrientation().setYawPitchRoll(Math.PI / 2.0, 0.0, Math.PI / 4.0);
               leftHandPanelPose.getPosition().addY(-0.05);
               leftHandPanelPose.changeFrame(ReferenceFrame.getWorldFrame());
               leftHandPanel.updateDesiredPose(leftHandPanelPose::get);
            }
         });
      }

      switch (mode)
      {
         case FOOTSTEP_PLACEMENT -> handPlacedFootstepMode.processVRInput(vrContext);
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
      vrModeControls.update();
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
      if (ImGui.radioButton(labels.get(RDXVRMode.WHOLE_BODY_IK_STREAMING.getReadableName()), mode == RDXVRMode.WHOLE_BODY_IK_STREAMING))
      {
         mode = RDXVRMode.WHOLE_BODY_IK_STREAMING;
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
   }

   public void destroy()
   {
      leftHandPanel.dispose();
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

   public RDX3DSituatedImGuiPanel getLeftHandPanel()
   {
      return leftHandPanel;
   }
}