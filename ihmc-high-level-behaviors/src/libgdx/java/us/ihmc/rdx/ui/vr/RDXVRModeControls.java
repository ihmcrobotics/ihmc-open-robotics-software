package us.ihmc.rdx.ui.vr;

import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;

public class RDXVRModeControls
{
   private static final String PANEL_NAME = "VR mode controls";
   private final RDXBaseUI baseUI;
   private final RDXVRModeManager vrModeManager;
   private boolean wasVRReady = false;

   public RDXVRModeControls(RDXBaseUI baseUI, RDXVRModeManager vrModeManager)
   {
      this.baseUI = baseUI;
      this.vrModeManager = vrModeManager;
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
