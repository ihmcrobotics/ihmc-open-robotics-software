package us.ihmc.rdx.ui.vr;

import imgui.ImGui;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiTransparentPanel;
import us.ihmc.rdx.imgui.RDXImGuiWindowAndDockSystem;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.rdx.tools.RDXIconTexture;

import java.util.ArrayList;

public class RDXVRAssistanceMenu
{
   private final FramePose3D menuPanelPose = new FramePose3D();
   private final RigidBodyTransform menuPanelTransformToWorld = new RigidBodyTransform();
   private ReferenceFrame menuPanelFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                                  menuPanelTransformToWorld);
   private RDX3DSituatedImGuiTransparentPanel menuPanel;
   private static final double[] positionPanel = {0.2, -0.1, 0.05};
   private final ArrayList<RDXIconTexture> iconsAssistanceMode = new ArrayList<>();
   private final ArrayList<RDXIconTexture> iconsJoysticks = new ArrayList<>();
   private final VRMenuGuideMode[] mode;


   public RDXVRAssistanceMenu(RDXImGuiWindowAndDockSystem window, VRMenuGuideMode[] mode)
   {
      menuPanel = new RDX3DSituatedImGuiTransparentPanel("Menu VR", this::renderMenu);
      menuPanel.create(window.getImGuiGl3(), 0.083, 0.07, 50);

      iconsAssistanceMode.add(new RDXIconTexture("icons/vrAssistance/" + "assistanceOff.png"));
      iconsAssistanceMode.add(new RDXIconTexture("icons/vrAssistance/" + "assistanceOn.png"));

      iconsJoysticks.add(new RDXIconTexture("icons/vrAssistance/" + "controllers.png"));
      iconsJoysticks.add(new RDXIconTexture("icons/vrAssistance/" + "pressBLeft.png"));
      iconsJoysticks.add(new RDXIconTexture("icons/vrAssistance/" + "moveRight.png"));
      iconsJoysticks.add(new RDXIconTexture("icons/vrAssistance/" + "pushLeftJoystick.png"));

      this.mode = mode;
   }

   public void update(ReferenceFrame placementFrame)
   {
      if (menuPanel != null)
      {
         menuPanelPose.setToZero(placementFrame);
         menuPanelPose.getPosition().set(positionPanel);
         menuPanelPose.changeFrame(ReferenceFrame.getWorldFrame());
         menuPanelPose.get(menuPanelTransformToWorld);
         menuPanelFrame.update();

         menuPanel.setTransformToReferenceFrame(menuPanelFrame);
         menuPanel.update();
      }
   }

   private void renderMenu() {
      ImGui.sameLine(ImGui.getCursorPosX() + ImGui.getContentRegionAvail().x - 92.7f - 95f);
      if (! (mode[0].equals(VRMenuGuideMode.OFF) || mode[0].equals(VRMenuGuideMode.PRESS_LEFT_B))) // if assistance off
         ImGui.image(iconsAssistanceMode.get(1).getTexture().getTextureObjectHandle(), 92.7f, 119.5f);
      else // assistance on
         ImGui.image(iconsAssistanceMode.get(0).getTexture().getTextureObjectHandle(), 92.7f, 119.5f);
      ImGui.newLine();
      ImGui.newLine();
      if (mode[0].equals(VRMenuGuideMode.IDLE))
         ImGui.image(iconsJoysticks.get(0).getTexture().getTextureObjectHandle(), 400.2f, 172.7f);
      else if (mode[0].equals(VRMenuGuideMode.PRESS_LEFT_B))
         ImGui.image(iconsJoysticks.get(1).getTexture().getTextureObjectHandle(), 400.2f, 172.7f);
      else if (mode[0].equals(VRMenuGuideMode.MOVE_RIGHT))
         ImGui.image(iconsJoysticks.get(2).getTexture().getTextureObjectHandle(), 400.2f, 172.7f);
      else if (mode[0].equals(VRMenuGuideMode.PUSH_LEFT_JOYSTICK))
         ImGui.image(iconsJoysticks.get(3).getTexture().getTextureObjectHandle(), 400.2f, 172.7f);
   }

   public RDX3DSituatedImGuiTransparentPanel getPanel()
   {
      return menuPanel;
   }
}
