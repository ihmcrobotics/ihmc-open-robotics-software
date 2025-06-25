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
import us.ihmc.behaviors.tools.interfaces.LogToolsLogger;
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
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.teleoperation.RDXHandConfigurationManager;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationManager;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRManager;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;
import java.util.Collection;
import java.util.Set;

public class RDXVRModeManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private RDXVRManager vrManager;
   private RDXVRMode mode = RDXVRMode.INPUTS_DISABLED;
   private ImBoolean interactablesEnabled;

   private RDXVRStereoVision stereoVision;
   private RDXVRFootstepPlacement footstepPlacer;
   private RDXHandConfigurationManager handManager;

   @Nullable
   private RDXVRWholeBodyKinematicStreaming kinematicsStreaming;
   private RDXVRFootstepStreaming footstepStreaming;
   private RDXJoystickBasedStepping joystickBasedStepping;

   private RDXVRModeControls vrModeControls;
   private RDX3DSituatedImGuiPanel vrModeControls3DPanel;
   private final FramePose3D vrModeControls3DPanelPose = new FramePose3D();


   public void create(RDXBaseUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      RDXROS2RobotVisualizer robotVisualizer,
                      ROS2ControllerHelper controllerHelper,
                      RetargetingParameters retargetingParameters,
                      boolean createKinematicsStreamingToolboxModule,
                      KinematicsStreamingToolboxParameters kstParameters,
                      FullHumanoidRobotModel miniGhostFullRobotModel)
   {
      vrManager = baseUI.getVRManager();

      Collection<RDXPanel> baseUIPanels =  RDXBaseUI.getInstance().getImGuiPanelManager().getPanels();
      for (RDXPanel panel : baseUIPanels)
      {
         if (panel instanceof RDXTeleoperationManager teleoperationPanel)
         {
            interactablesEnabled = teleoperationPanel.getInteractablesEnabled();
//            handManager = teleoperationPanel.getArmManager().getHandConfigurationManager();
            break;
         }
      }
      footstepPlacer = new RDXVRFootstepPlacement(baseUI.getVRManager().getContext(), syncedRobot, controllerHelper);
      stereoVision = new RDXVRStereoVision(syncedRobot.getReferenceFrames());

      if (syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.LEFT) ||
          syncedRobot.getRobotModel().getRobotVersion().hasArm(RobotSide.RIGHT))
      {
         kinematicsStreaming = new RDXVRWholeBodyKinematicStreaming(syncedRobot,
                                                                    controllerHelper,
                                                                    robotVisualizer,
                                                                    vrManager.getContext(),
                                                                    retargetingParameters,
                                                                    kstParameters,
                                                                    createKinematicsStreamingToolboxModule,
                                                                    handManager,
                                                                    miniGhostFullRobotModel);
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
      if (mode != RDXVRMode.INPUTS_DISABLED)
         interactablesEnabled.set(false);

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
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.radioButton(labels.get(RDXVRMode.INPUTS_DISABLED.getReadableName()), mode == RDXVRMode.INPUTS_DISABLED))
      {
         mode = RDXVRMode.INPUTS_DISABLED;
         if (kinematicsStreaming != null)
            kinematicsStreaming.setKSTEnabled(false);
         footstepPlacer.reset();
         footstepStreaming.reset();
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.FOOTSTEP_PLACEMENT.getReadableName()), mode == RDXVRMode.FOOTSTEP_PLACEMENT))
      {
         mode = RDXVRMode.FOOTSTEP_PLACEMENT;
         if (kinematicsStreaming != null)
            kinematicsStreaming.setKSTEnabled(false);
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.FOOTSTEP_STREAMING.getReadableName()), mode == RDXVRMode.FOOTSTEP_STREAMING))
      {
         mode = RDXVRMode.FOOTSTEP_STREAMING;
         if (kinematicsStreaming != null)
            kinematicsStreaming.setKSTEnabled(false);
         footstepStreaming.reset();
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
      }
      if (kinematicsStreaming == null)
      {
         ImGui.popStyleColor();
      }
      if (ImGui.radioButton(labels.get(RDXVRMode.JOYSTICK_WALKING.getReadableName()), mode == RDXVRMode.JOYSTICK_WALKING))
      {
         mode = RDXVRMode.JOYSTICK_WALKING;
         if (kinematicsStreaming != null)
            kinematicsStreaming.setKSTEnabled(false);
         footstepPlacer.reset();
      }
   }

   public void render()
   {
      stereoVision.renderProjection();
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

         if (stereoVision.isEnabled())
            stereoVision.getDualBlackflySphericalProjection().getRenderables(renderables, pool, sceneLevels);

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
      stereoVision.getDualBlackflySphericalProjection().shutdown();
      footstepPlacer.destroy();
      footstepStreaming.destroy();
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

   public RDXVRStereoVision getStereoVision()
   {
      return stereoVision;
   }
}