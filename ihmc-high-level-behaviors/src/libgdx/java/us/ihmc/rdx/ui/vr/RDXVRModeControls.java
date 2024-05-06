package us.ihmc.rdx.ui.vr;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;

public class RDXVRModeControls
{
   private static final String PANEL_NAME = "VR mode controls";
   private final RDXVRModeManager vrModeManager;
   private boolean wasVRReady = false;
   private final ImBoolean renderOnLeftHand = new ImBoolean(false);

   public RDXVRModeControls(RDXVRModeManager vrModeManager)
   {
      this.vrModeManager = vrModeManager;
   }

   public void update()
   {
      RDXBaseUI baseUI = RDXBaseUI.getInstance();

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

   public void render()
   {
      ImGui.checkbox("Render on left hand", renderOnLeftHand);

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
            case WHOLE_BODY_IK_STREAMING -> {
               if (vrModeManager.getKinematicsStreamingMode() != null)
               {
                  vrModeManager.getKinematicsStreamingMode().renderImGuiWidgets();
               }
               else
               {
                  ImGui.text("Kinematics streaming mode not available");
               }
            }
            case JOYSTICK_WALKING -> vrModeManager.getJoystickBasedStepping().renderImGuiWidgets();
         }
      }
   }

   public ImBoolean getRenderOnLeftHand()
   {
      return renderOnLeftHand;
   }
}
