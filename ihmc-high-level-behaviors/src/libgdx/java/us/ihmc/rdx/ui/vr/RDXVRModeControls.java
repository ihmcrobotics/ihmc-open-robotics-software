package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRModeControls
{
   private static final String PANEL_NAME = "VR mode controls";
   private final RDXBaseUI baseUI;
   private final RDXVRModeManager vrModeManager;
   private final RDX3DSituatedImGuiPanel leftHandPanel;
   private final FramePose3D leftHandPanelPose = new FramePose3D();
   private boolean wasVRReady = false;

   public RDXVRModeControls(RDXBaseUI baseUI, RDXVRModeManager vrModeManager)
   {
      this.baseUI = baseUI;
      this.vrModeManager = vrModeManager;

      leftHandPanel = new RDX3DSituatedImGuiPanel(PANEL_NAME, this::render);
      leftHandPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
      leftHandPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
      baseUI.getVRManager().getContext().addVRPickCalculator(leftHandPanel::calculateVRPick);
      baseUI.getVRManager().getContext().addVRInputProcessor(leftHandPanel::processVRInput);
      baseUI.getVRManager().getContext().addVRInputProcessor(this::processVRInput);
      baseUI.getPrimaryScene().addModelInstance(leftHandPanel.getModelInstance());
      baseUI.getPrimary3DPanel().addOverlayPanel(PANEL_NAME, this::render);
   }

   public void update()
   {
      boolean isVRReady = baseUI.getVRManager().isVRReady();

      if (isVRReady && !wasVRReady)
      {
         baseUI.getPrimary3DPanel().addOverlayPanel(PANEL_NAME, this::render);
      }
      else if (!isVRReady && wasVRReady)
      {
         baseUI.getPrimary3DPanel().removeOverlayPanel(PANEL_NAME);
      }

      wasVRReady = isVRReady;

      leftHandPanel.update();
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
   }

   private void render()
   {
      if (true)
      {
         ImGuiTools.separatorText("Stereo vision");
         vrModeManager.getStereoVision().renderControls();

         ImGuiTools.separatorText("Mode");
         vrModeManager.renderImGuiWidgets();

         ImGuiTools.separatorText("Controls");
         RDXBaseUI.getInstance().getKeyBindings().renderKeybindingsSection(RDXVRModeManager.class.getSimpleName());
         switch (vrModeManager.getMode())
         {
            case FOOTSTEP_PLACEMENT -> RDXBaseUI.getInstance().getKeyBindings().renderKeybindingsSection(RDXVRHandPlacedFootstepMode.class.getSimpleName());
            case WHOLE_BODY_IK_STREAMING ->
                  RDXBaseUI.getInstance().getKeyBindings().renderKeybindingsSection(RDXVRKinematicsStreamingMode.class.getSimpleName());
            case JOYSTICK_WALKING -> RDXBaseUI.getInstance().getKeyBindings().renderKeybindingsSection(RDXJoystickBasedStepping.class.getSimpleName());
         }

         // Render options for VR modes that have options
         if (vrModeManager.getMode() == RDXVRMode.WHOLE_BODY_IK_STREAMING || vrModeManager.getMode() == RDXVRMode.JOYSTICK_WALKING)
         {
            ImGuiTools.separatorText(vrModeManager.getMode().getReadableName() + " options");

            switch (vrModeManager.getMode())
            {
               case WHOLE_BODY_IK_STREAMING -> vrModeManager.getKinematicsStreamingMode().renderImGuiWidgets();
               case JOYSTICK_WALKING -> vrModeManager.getJoystickBasedStepping().renderImGuiWidgets();
            }
         }
      }
   }
}
