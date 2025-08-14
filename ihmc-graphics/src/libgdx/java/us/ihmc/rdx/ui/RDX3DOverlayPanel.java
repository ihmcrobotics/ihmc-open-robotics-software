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
   private float lastPanelWidth = -1, lastPanelHeight = -1, lastStartX = -1, lastStartY = -1;
   private float lastBgAlpha = -1;

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

      float startX = parent.getWindowPositionX() + (parent.getWindowSizeX() - panelWidth - 5);
      float startY = previousActiveWindowY + 10;
      float bgAlpha = windowActiveLerp;

      if (panelWidth != lastPanelWidth || panelHeight != lastPanelHeight)
         ImGui.setNextWindowSize(panelWidth, panelHeight);
      if (startX != lastStartX || startY != lastStartY)
         ImGui.setNextWindowPos(startX, startY);
      if (bgAlpha != lastBgAlpha)
         ImGui.setNextWindowBgAlpha(bgAlpha);

      lastPanelWidth = panelWidth;
      lastPanelHeight = panelHeight;
      lastStartX = startX;
      lastStartY = startY;
      lastBgAlpha = bgAlpha;

      int windowFlags = ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoCollapse | ImGuiWindowFlags.NoMove;
      ImGui.begin(labels.get(panelName), windowFlags);


      boolean hovered = ImGui.isWindowHovered(
            ImGuiHoveredFlags.ChildWindows | ImGuiHoveredFlags.AllowWhenBlockedByActiveItem | ImGuiHoveredFlags.AllowWhenBlockedByPopup);

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
