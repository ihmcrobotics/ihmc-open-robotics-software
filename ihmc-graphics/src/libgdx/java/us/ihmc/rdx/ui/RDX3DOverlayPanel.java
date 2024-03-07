package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDX3DOverlayPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final String panelName;
   private final Runnable imGuiRender;
   private float windowActiveLerp;
   private final RDX3DPanel parent;

   public RDX3DOverlayPanel(String panelName, Runnable imGuiRender, RDX3DPanel parent)
   {
      this.panelName = panelName;
      this.imGuiRender = imGuiRender;
      this.parent = parent;
   }

   public float render(int panelIndex, float previousActiveWindowLerp)
   {
      float panelWidth = 400;
      float panelHeight = 300 * windowActiveLerp;

      ImGui.setNextWindowSize(panelWidth, panelHeight);
      float startX = parent.getWindowPositionX() + (parent.getWindowSizeX() - panelWidth - 5);
      float startY = (parent.getWindowPositionY() + 10) + (panelIndex * 310 * previousActiveWindowLerp);
      ImGui.setNextWindowPos(startX, startY);
      ImGui.setNextWindowBgAlpha(windowActiveLerp);
      int windowFlags = ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse;
      ImGui.begin(labels.get(panelName), windowFlags);

      if (ImGui.isWindowHovered())
      {
         windowActiveLerp = 1.0f;
      }
      else
      {
         windowActiveLerp = (float) InterpolationTools.linearInterpolate(windowActiveLerp, 0.2f, 0.01f);
      }

      imGuiRender.run();

      ImGui.end();

      return windowActiveLerp;
   }

   public String getPanelName()
   {
      return panelName;
   }
}
