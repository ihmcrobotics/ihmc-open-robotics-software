package us.ihmc.rdx.ui.vr;

import imgui.ImGui;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXJoystickBasedStepping;

public class RDXVRModeControls
{
   private final RDX3DPanel panel;
   private final RDXVRModeManager vrModeManager;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXVRModeControls(RDX3DPanel panel, RDXVRModeManager vrModeManager)
   {
      this.panel = panel;
      this.vrModeManager = vrModeManager;
   }

   public void render()
   {
      float panelWidth = 400;
      float panelHeight = 300;

      ImGui.setNextWindowSize(panelWidth, panelHeight);
      float startX = panel.getWindowPositionX() + (panel.getWindowSizeX() - panelWidth - 5);
      float startY = (panel.getWindowPositionY() + 10);
      ImGui.setNextWindowPos(startX, startY);
      ImGui.setNextWindowBgAlpha(0.8f);
      int windowFlags = ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoTitleBar; // undecorated
      ImGui.begin(labels.get("VRModeControls"), windowFlags);

      ImGuiTools.textBold("VR");
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

      if (vrModeManager.getMode() != RDXVRMode.INPUTS_DISABLED)
      {
         ImGuiTools.separatorText(vrModeManager.getMode().getReadableName() + " options");

         switch (vrModeManager.getMode())
         {
            case FOOTSTEP_PLACEMENT -> vrModeManager.getHandPlacedFootstepMode().renderImGuiWidgets();
            case WHOLE_BODY_IK_STREAMING -> vrModeManager.getKinematicsStreamingMode().renderImGuiWidgets();
            case JOYSTICK_WALKING -> vrModeManager.getJoystickBasedStepping().renderImGuiWidgets();
         }
      }

      ImGui.end();
   }
}
