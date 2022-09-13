package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.GDX3DSituatedImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.GDXJoystickBasedStepping;
import us.ihmc.gdx.ui.graphics.GDX3DSituatedImagePanel;
import us.ihmc.gdx.ui.missionControl.processes.RestartableJavaProcess;
import us.ihmc.gdx.ui.teleoperation.GDXTeleoperationParameters;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.function.Supplier;

/**
 * TODO: Figure out how to incorporate this class with things better.
 */
public class GDXVRModeManager
{
   private GDXImGuiBasedUI baseUI;
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
   private GDX3DSituatedImagePanel floatingVideoPanel;
   private final ModifiableReferenceFrame floatingPanelFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private final FramePose3D floatingPanelFramePose = new FramePose3D();
   private final RigidBodyTransform gripOffsetTransform = new RigidBodyTransform();
   private boolean grippedLastTime = false;
   private Supplier<Texture> floatingVideoPanelTextureSupplier;
   private boolean modeChangedThisUpdate = false;

   public void create(GDXImGuiBasedUI baseUI,
                      ROS2SyncedRobotModel syncedRobot,
                      ROS2ControllerHelper controllerHelper,
                      RestartableJavaProcess kinematicsStreamingToolboxProcess,
                      GDXTeleoperationParameters teleoperationParameters)
   {
      this.baseUI = baseUI;
      this.syncedRobot = syncedRobot;
      handPlacedFootstepMode = new GDXVRHandPlacedFootstepMode();
      handPlacedFootstepMode.create(syncedRobot.getRobotModel(), controllerHelper);
      handPlacedFootstepMode.setTeleoperationParameters(teleoperationParameters);

      if (kinematicsStreamingToolboxProcess != null)
      {
         kinematicsStreamingMode = new GDXVRKinematicsStreamingMode(syncedRobot.getRobotModel(), controllerHelper, kinematicsStreamingToolboxProcess);
         kinematicsStreamingMode.create(baseUI.getVRManager().getContext());
      }

      joystickBasedStepping = new GDXJoystickBasedStepping(syncedRobot.getRobotModel());
      joystickBasedStepping.create(baseUI, controllerHelper, syncedRobot);

      baseUI.getImGuiPanelManager().addPanel("VR Mode Manager", this::renderImGuiWidgets);

      leftHandPanel = new GDX3DSituatedImGuiPanel("VR Mode Manager", this::renderImGuiWidgets);
      leftHandPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
      leftHandPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
      baseUI.getVRManager().getContext().addVRPickCalculator(leftHandPanel::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(leftHandPanel::processVRInput);

      baseUI.getPrimaryScene().addRenderableProvider(this::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
      baseUI.getVRManager().getContext().addVRInputProcessor(this::processVRInput);
   }

   public void setVideoPanelTexture(Supplier<Texture> textureSupplier)
   {
      floatingVideoPanelTextureSupplier = textureSupplier;
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

   public void update(boolean nativesLoaded, boolean nativesNewlyLoaded)
   {
      if (nativesLoaded)
      {
         if (nativesNewlyLoaded)
         {
            floatingVideoPanel = new GDX3DSituatedImagePanel();
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) ->
            {
               if (showFloatingVideoPanel.get())
               {
                  floatingVideoPanel.getRenderables(renderables, pool);
               }
            }, GDXSceneLevel.VIRTUAL);
            baseUI.getVRManager().getContext().addVRInputProcessor(vrContext ->
            {
               vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
               {
                  if (floatingVideoPanel.getModelInstance() != null)
                  {
                     // GDXTools.toEuclid(floatingVideoPanel.getModelInstance().transform, floatingPanelFrame.getTransformToParent());
                     // floatingPanelFrame.getReferenceFrame().update();
                     floatingPanelFramePose.setToZero(floatingPanelFrame.getReferenceFrame());
                     floatingPanelFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                     boolean controllerIsCloseToPanel = controller.getXForwardZUpPose().getPosition().distance(floatingPanelFramePose.getPosition()) < 0.05;
                     boolean isGripping = controller.getGripActionData().x() > 0.9;
                     if ((grippedLastTime || controllerIsCloseToPanel) && isGripping)
                     {
                        if (!grippedLastTime) // set up offset
                        {
                           floatingPanelFramePose.changeFrame(controller.getXForwardZUpControllerFrame());
                           floatingPanelFramePose.get(gripOffsetTransform);
                           floatingPanelFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                        }

                        // tempPanelTransform.set(gripOffsetTransform);
                        // controller.getXForwardZUpControllerFrame().getTransformToWorldFrame().transform(tempPanelTransform);
                        // GDXTools.toGDX(tempPanelTransform, floatingVideoPanel.getModelInstance().transform);
                        // floatingPanelFrame.getTransformToParent().set(tempPanelTransform);
                        floatingPanelFrame.getTransformToParent().set(gripOffsetTransform);
                        controller.getXForwardZUpControllerFrame().getTransformToWorldFrame().transform(floatingPanelFrame.getTransformToParent());
                        floatingPanelFrame.getReferenceFrame().update();

                        grippedLastTime = true;
                     }
                     else
                     {
                        grippedLastTime = false;
                     }
                  }
               });
               vrContext.getHeadset().runIfConnected(headset ->
               {
                  if (floatingVideoPanel.getModelInstance() != null && showFloatVideoPanelNotification.poll())
                  {
                     floatingPanelFramePose.setToZero(headset.getXForwardZUpHeadsetFrame());
                     floatingPanelFramePose.getPosition().set(0.5, -0.2, 0.0);
                     floatingPanelFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                     floatingPanelFramePose.get(floatingPanelFrame.getTransformToParent());
                     floatingPanelFrame.getReferenceFrame().update();
                  }
               });
            });
         }

         if (showFloatingVideoPanel.get())
         {
            if (floatingVideoPanelTextureSupplier.get() != floatingVideoPanel.getTexture())
            {
               boolean flipY = false;
               float multiplier = 2.0f;
               //                     float halfWidth = 0.0848f * multiplier;
               //                     float halfHeight = 0.0480f * multiplier;
               float halfWidth = 0.1920f * multiplier;
               float halfHeight = 0.1200f * multiplier;
               floatingVideoPanel.create(floatingVideoPanelTextureSupplier.get(),
                                         new Vector3[] {new Vector3(0.0f, halfWidth, halfHeight),
                                                        new Vector3(0.0f, halfWidth, -halfHeight),
                                                        new Vector3(0.0f, -halfWidth, -halfHeight),
                                                        new Vector3(0.0f, -halfWidth, halfHeight)},
                                         floatingPanelFrame.getReferenceFrame(),
                                         flipY);
            }
            floatingVideoPanel.setPoseToReferenceFrame(floatingPanelFrame.getReferenceFrame());
         }
      }

      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.update(mode == GDXVRMode.WHOLE_BODY_IK_STREAMING);
      leftHandPanel.update();
      joystickBasedStepping.update(mode == GDXVRMode.JOYSTICK_WALKING);
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Teleport: Right B button");
      ImGui.text("Adjust user Z height: Right touchpad up/down");
      ImGui.text("ImGui panels: Point and use right trigger to click and drag");
      if (floatingVideoPanelTextureSupplier != null && ImGui.checkbox(labels.get("Floating video panel"), showFloatingVideoPanel))
      {
         if (showFloatingVideoPanel.get())
            showFloatVideoPanelNotification.set();
      }
      GDXVRMode previousMode = mode;
      if (ImGui.radioButton(labels.get("Inputs disabled"), mode == GDXVRMode.INPUTS_DISABLED))
      {
         mode = GDXVRMode.INPUTS_DISABLED;
      }
      if (ImGui.radioButton(labels.get("Footstep placement"), mode == GDXVRMode.FOOTSTEP_PLACEMENT))
      {
         mode = GDXVRMode.FOOTSTEP_PLACEMENT;
      }
      if (kinematicsStreamingMode != null && ImGui.radioButton(labels.get("Whole body IK streaming"), mode == GDXVRMode.WHOLE_BODY_IK_STREAMING))
      {
         mode = GDXVRMode.WHOLE_BODY_IK_STREAMING;
      }
      if (ImGui.radioButton(labels.get("Joystick walking"), mode == GDXVRMode.JOYSTICK_WALKING))
      {
         mode = GDXVRMode.JOYSTICK_WALKING;
      }
      modeChangedThisUpdate = mode != previousMode;

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
      if (kinematicsStreamingMode != null)
         kinematicsStreamingMode.destroy();
   }

   public GDXVRMode getMode()
   {
      return mode;
   }

   public boolean getModeChangedThisUpdate()
   {
      return modeChangedThisUpdate;
   }
}
