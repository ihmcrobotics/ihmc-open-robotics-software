package us.ihmc.rdx.ui.vr;

import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;

public class RDXVRModeControls
{
   private final RDXVRModeManager vrModeManager;

   public RDXVRModeControls(RDXBaseUI baseUI, RDXVRModeManager vrModeManager)
   {
      this.vrModeManager = vrModeManager;
      baseUI.getPrimary3DPanel().addOverlayPanel("VR mode controls", this::render);
      baseUI.getPrimary3DPanel().addOverlayPanel("VR mode controls 2", this::render);
      baseUI.getPrimary3DPanel().addOverlayPanel("VR mode controls 3", this::render);
   }

   private void render()
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
         case WHOLE_BODY_IK_STREAMING -> RDXBaseUI.getInstance().getKeyBindings().renderKeybindingsSection(RDXVRKinematicsStreamingMode.class.getSimpleName());
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
