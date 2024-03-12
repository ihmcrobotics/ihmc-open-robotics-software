package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiHoveredFlags;
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

   public float render(float previousActiveWindowY)
   {
      float panelWidth = 400;
      float panelHeight = 300 * windowActiveLerp;

      ImGui.setNextWindowSize(panelWidth, panelHeight);
      float startX = parent.getWindowPositionX() + (parent.getWindowSizeX() - panelWidth - 5);
      float startY = previousActiveWindowY + 10;
      ImGui.setNextWindowPos(startX, startY);
      ImGui.setNextWindowBgAlpha(windowActiveLerp);
      int windowFlags = ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse;
      ImGui.begin(labels.get(panelName), windowFlags);

      if (ImGui.isWindowHovered(ImGuiHoveredFlags.ChildWindows | ImGuiHoveredFlags.AllowWhenBlockedByActiveItem | ImGuiHoveredFlags.AllowWhenBlockedByPopup))
      {
         windowActiveLerp = 0.9f;
      }
      else
      {
         windowActiveLerp = (float) InterpolationTools.linearInterpolate(windowActiveLerp, 0.2f, 0.05f);
      }

      imGuiRender.run();

      ImGui.end();

      return startY + panelHeight;
   }

   public String getPanelName()
   {
      return panelName;
   }
}
