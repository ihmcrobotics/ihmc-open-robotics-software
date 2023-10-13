package us.ihmc.rdx.ui.vr;

import imgui.ImGui;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
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
   private static final double[] positionPanel = {0.0, -0.1, 0.08};
   private static final YawPitchRoll yawPitchRollPanel = new YawPitchRoll(-Math.toRadians(45),Math.toRadians(45),0);
   private final ArrayList<RDXIconTexture> iconsAssistanceMode = new ArrayList<>();
   private final ArrayList<RDXIconTexture> iconsAssistanceProgress = new ArrayList<>();
   private final ArrayList<RDXIconTexture> iconsJoysticks = new ArrayList<>();
   private final ArrayList<RDXIconTexture> iconsJoysticksHighlighted = new ArrayList<>();
   private boolean isHighlighted = false;
   private long lastRenderTime = System.currentTimeMillis();
   private final RDXVRMenuGuideMode[] mode;

   private int proMPSamples = -1;
   private int currentProMPSample = -1;
   private boolean hasProMP = false;
   private int affordanceSamples = -1;
   private int currentAffordanceSample = -1;
   private boolean hasAffordance = false;

   public RDXVRAssistanceMenu(RDXImGuiWindowAndDockSystem window, RDXVRMenuGuideMode[] mode)
   {
      menuPanel = new RDX3DSituatedImGuiTransparentPanel("Menu VR", this::renderMenu);
      menuPanel.create(window.getImGuiGl3(), 0.083, 0.072, 50);

      iconsAssistanceMode.add(new RDXIconTexture("icons/vrAssistance/" + "assistanceOff.png"));
      iconsAssistanceMode.add(new RDXIconTexture("icons/vrAssistance/" + "assistanceOn.png"));

      iconsJoysticks.add(new RDXIconTexture("icons/vrAssistance/" + "controllers.png"));
      iconsJoysticks.add(new RDXIconTexture("icons/vrAssistance/" + "pressBLeft.png"));
      iconsJoysticks.add(new RDXIconTexture("icons/vrAssistance/" + "move.png"));
      iconsJoysticks.add(new RDXIconTexture("icons/vrAssistance/" + "pushLeftJoystick.png"));
      iconsJoysticks.add(new RDXIconTexture("icons/vrAssistance/" + "pressLeftAorB.png"));

      iconsJoysticksHighlighted.add(new RDXIconTexture("icons/vrAssistance/" + "controllers.png"));
      iconsJoysticksHighlighted.add(new RDXIconTexture("icons/vrAssistance/" + "pressBLeftHighlight.png"));
      iconsJoysticksHighlighted.add(new RDXIconTexture("icons/vrAssistance/" + "moveHighlight.png"));
      iconsJoysticksHighlighted.add(new RDXIconTexture("icons/vrAssistance/" + "pushLeftJoystickHighlight.png"));
      iconsJoysticksHighlighted.add(new RDXIconTexture("icons/vrAssistance/" + "pressLeftAorBHighlight.png"));

      for (int i = 0; i <= 100; i += 5)
         iconsAssistanceProgress.add(new RDXIconTexture("icons/vrAssistance/progressBar/" + i + ".png"));

      this.mode = mode;
   }

   public void update(ReferenceFrame placementFrame)
   {
      if (menuPanel != null)
      {
         menuPanelPose.setToZero(placementFrame);
         menuPanelPose.getPosition().set(positionPanel);
         menuPanelPose.getOrientation().set(yawPitchRollPanel);
         menuPanelPose.changeFrame(ReferenceFrame.getWorldFrame());
         menuPanelPose.get(menuPanelTransformToWorld);
         menuPanelFrame.update();

         menuPanel.setTransformToReferenceFrame(menuPanelFrame);
         menuPanel.update();
      }
   }

   private void renderMenu()
   {
      long currentTime = System.currentTimeMillis();
      if (currentTime - lastRenderTime >= 500)
      {
         isHighlighted = !isHighlighted;
         lastRenderTime = currentTime;
      }

      if (!(mode[0] == RDXVRMenuGuideMode.OFF || mode[0] == RDXVRMenuGuideMode.ACTIVATE)) // assistance on
      {
         if (mode[0] == RDXVRMenuGuideMode.DIRECT_WITH_JOYSTICK)
         {
            if (hasProMPSamples())
            {
               hasProMP = true;
               float maxPercentage = hasAffordance ? 50.0f : 100.0f;
               int percentageProgress = currentProMPSample < 0 ? 0 : Math.round((currentProMPSample * maxPercentage) / proMPSamples / 5.0f);
               ImGui.sameLine(ImGui.getCursorPosX() + 10.0f);
               ImGui.image(iconsAssistanceProgress.get(percentageProgress).getTexture().getTextureObjectHandle(), 250.1f, 121.3f);
            }
            else if (hasAffordanceSamples())
            {
               int start = hasProMP ? 50 : 0;
               float maxPercentage = hasProMP ? 50.0f : 100.0f;
               int percentageProgress =
                     currentAffordanceSample < 0 ? start : Math.round((start + (currentAffordanceSample * (maxPercentage)) / affordanceSamples) / 5.0f);
               ImGui.sameLine(ImGui.getCursorPosX() + 10.0f);
               ImGui.image(iconsAssistanceProgress.get(percentageProgress).getTexture().getTextureObjectHandle(), 250.1f, 121.3f);
            }
         }
         ImGui.sameLine(ImGui.getCursorPosX() + ImGui.getContentRegionAvail().x - 92.7f - 45f);
         ImGui.image(iconsAssistanceMode.get(1).getTexture().getTextureObjectHandle(), 112.0f, 135.6f);
      }
      else // assistance off
      {
         ImGui.sameLine(ImGui.getCursorPosX() + ImGui.getContentRegionAvail().x - 92.7f - 45f);
         ImGui.image(iconsAssistanceMode.get(0).getTexture().getTextureObjectHandle(), 112.0f, 135.6f);
         hasProMP = false;
      }
      ImGui.newLine();
      ImGui.newLine();
      if (mode[0] == RDXVRMenuGuideMode.IDLE)
         ImGui.image(iconsJoysticks.get(0).getTexture().getTextureObjectHandle(), 400.2f, 172.7f);
      else if (mode[0] == RDXVRMenuGuideMode.ACTIVATE)
         ImGui.image(isHighlighted ?
                           iconsJoysticksHighlighted.get(1).getTexture().getTextureObjectHandle() :
                           iconsJoysticks.get(1).getTexture().getTextureObjectHandle(), 400.2f, 172.7f);
      else if (mode[0] == RDXVRMenuGuideMode.MOVE)
         ImGui.image(isHighlighted ?
                           iconsJoysticksHighlighted.get(2).getTexture().getTextureObjectHandle() :
                           iconsJoysticks.get(2).getTexture().getTextureObjectHandle(), 400.2f, 172.7f);
      else if (mode[0] == RDXVRMenuGuideMode.DIRECT_WITH_JOYSTICK)
         ImGui.image(isHighlighted ?
                           iconsJoysticksHighlighted.get(3).getTexture().getTextureObjectHandle() :
                           iconsJoysticks.get(3).getTexture().getTextureObjectHandle(), 400.2f, 172.7f);
      else if (mode[0] == RDXVRMenuGuideMode.VALIDATE)
         ImGui.image(isHighlighted ?
                           iconsJoysticksHighlighted.get(4).getTexture().getTextureObjectHandle() :
                           iconsJoysticks.get(4).getTexture().getTextureObjectHandle(), 400.2f, 172.7f);
   }

   public void resetTimer()
   {
      lastRenderTime = System.currentTimeMillis();
   }

   public void setJoystickPressed(boolean pressed)
   {
      if (pressed)
         isHighlighted = false;
   }

   public RDX3DSituatedImGuiTransparentPanel getPanel()
   {
      return menuPanel;
   }

   public boolean hasProMPSamples()
   {
      return proMPSamples >= 0;
   }

   public void setProMPSamples(int samples)
   {
      proMPSamples = samples;
      if (samples == -1)
         currentProMPSample = -1;
   }

   public void setCurrentProMPSample(int sample)
   {
      currentProMPSample = sample;
   }

   public boolean hasAffordanceSamples()
   {
      return affordanceSamples >= 0;
   }

   public void setAffordanceSamples(int samples)
   {
      affordanceSamples = samples;
      if (samples == -1)
         currentAffordanceSample = -1;
   }

   public void setCurrentAffordanceSample(int sample)
   {
      currentAffordanceSample = sample;
   }

   public void setHasAffordance(boolean hasAffordance)
   {
      this.hasAffordance = hasAffordance;
   }
}