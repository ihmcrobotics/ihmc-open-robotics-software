package us.ihmc.rdx.ui.vr;

import imgui.ImGui;
import imgui.flag.ImGuiTreeNodeFlags;
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
   private final ImBoolean showFloatingVideoPanel = new ImBoolean(false);
   private final ImBoolean useStereoVision = new ImBoolean(false);
   private final RDXBaseUI baseUI;

   public RDXVRModeControls(RDXVRModeManager vrModeManager)
   {
      this.vrModeManager = vrModeManager;
      baseUI = RDXBaseUI.getInstance();

      // Optionally accessible as a normal panel
      baseUI.getImGuiPanelManager().addPanel(PANEL_NAME, this::render);
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

   public void render()
   {
      ImGui.checkbox("Render on left hand", renderOnLeftHand);

      if (ImGui.collapsingHeader("VR Mode", ImGuiTreeNodeFlags.DefaultOpen))
      {
         vrModeManager.renderImGuiWidgets();
      }

      // Render options for VR modes that have options
      boolean optionsAvailable = false;
      if (vrModeManager.getMode() == RDXVRMode.WHOLE_BODY_IK_STREAMING || vrModeManager.getMode() == RDXVRMode.JOYSTICK_WALKING)
      {
         optionsAvailable = true;
      }
      else
      {
         ImGui.beginDisabled();
      }
      if (ImGui.collapsingHeader("VR Mode Options"))
      {
         // Render options for VR modes that have options
         if (optionsAvailable)
         {
            ImGuiTools.separatorText("Main Options", ImGuiTools.getSmallBoldFont());
            switch (vrModeManager.getMode())
            {
               case WHOLE_BODY_IK_STREAMING ->
               {
                  if (vrModeManager.getKinematicsStreaming() != null)
                  {
                     vrModeManager.getKinematicsStreaming().renderImGuiWidgets();
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
      if (!optionsAvailable)
      {
         ImGui.endDisabled();
      }

      if (ImGui.collapsingHeader("Visual Feedback"))
      {
         ImGui.checkbox("Floating Video Panel", showFloatingVideoPanel);
         if (showFloatingVideoPanel.get())
         {
            // Stereo checkbox is enabled and interactive
            ImGui.checkbox("Stereo Enabled", useStereoVision);
         }
         else
         {
            // Stereo checkbox is disabled (grayed out)
            ImGui.beginDisabled();
            ImGui.checkbox("Stereo Enabled", useStereoVision);
            ImGui.endDisabled();
         }
      }

      if (ImGui.collapsingHeader("Trackers"))
      {
         if (ImGui.button("Save roles"))
         {
            baseUI.getVRManager().getContext().saveTrackerRolesToFile();
         }
         ImGui.sameLine();
         if (ImGui.button("Load roles"))
         {
            baseUI.getVRManager().getContext().loadTrackerRolesFromFile();
         }
      }

      if (ImGui.collapsingHeader("Bindings"))
      {
         RDXBaseUI.getInstance().getKeyBindings().renderKeybindingsSection(RDXVRModeManager.class.getSimpleName());
         switch (vrModeManager.getMode())
         {
            case FOOTSTEP_PLACEMENT -> RDXBaseUI.getInstance().getKeyBindings().renderKeybindingsSection(RDXVRFootstepPlacement.class.getSimpleName());
            case WHOLE_BODY_IK_STREAMING ->
                  RDXBaseUI.getInstance().getKeyBindings().renderKeybindingsSection(RDXVRWholeBodyKinematicStreaming.class.getSimpleName());
            case JOYSTICK_WALKING -> RDXBaseUI.getInstance().getKeyBindings().renderKeybindingsSection(RDXJoystickBasedStepping.class.getSimpleName());
         }
      }
   }

   public ImBoolean getRenderOnLeftHand()
   {
      return renderOnLeftHand;
   }

   public ImBoolean getShowFloatingVideoPanel()
   {
      return showFloatingVideoPanel;
   }

   public ImBoolean getUseStereoVision()
   {
      return useStereoVision;
   }
}
