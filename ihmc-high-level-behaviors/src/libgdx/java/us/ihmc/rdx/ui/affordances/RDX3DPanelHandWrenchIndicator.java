package us.ihmc.rdx.ui.affordances;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDX3DPanelHandWrenchIndicator
{
   private final RDX3DPanel panel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final SideDependentList<RDX3DPanelHandWrenchIndicatorSide> sides = new SideDependentList<>();
   private final String contextMenuLabel = labels.get("Context Menu");

   public RDX3DPanelHandWrenchIndicator(RDX3DPanel panel)
   {
      this.panel = panel;

      for (RobotSide side : RobotSide.values)
      {
         sides.put(side, new RDX3DPanelHandWrenchIndicatorSide(side));
      }
   }

   public void renderImGuiOverlay()
   {
      // temporarily set arbitrary width height
      float panelWidth = 200;
      float panelHeight = ImGui.getFrameHeight(); // Start with space for separator

      for (RobotSide side : RobotSide.values)
      {
         panelHeight += sides.get(side).getHeight();
      }

      ImGui.setNextWindowSize(panelWidth, panelHeight);
      float startX = panel.getWindowPositionX() + (panel.getWindowSizeX() - panelWidth - 5);
      float startY = (panel.getWindowPositionY() + 10);
      ImGui.setNextWindowPos(startX, startY);
      ImGui.setNextWindowBgAlpha(0.2f);
      int windowFlags = ImGuiWindowFlags.NoTitleBar; // undecorated
      ImGui.begin(labels.get("WrenchMagnitudeIndicator"), windowFlags);
      for (RobotSide side : RobotSide.values)
      {
         sides.get(side).renderImGuiWidgets();
         if (side == RobotSide.LEFT)
            ImGui.separator();
      }

      if (ImGui.isWindowHovered() && ImGui.isMouseClicked(ImGuiMouseButton.Right))
      {
         ImGui.openPopup(contextMenuLabel);
      }
      if (ImGui.beginPopup(contextMenuLabel))
      {
         if (ImGui.menuItem(labels.get("Show Plots"), null, sides.get(RobotSide.LEFT).getShowPlots()))
         {
            boolean newValue = !sides.get(RobotSide.LEFT).getShowPlots();
            for (RobotSide side : RobotSide.values)
            {
               sides.get(side).setShowPlots(newValue);
            }
         }
         if (ImGui.menuItem("Cancel"))
            ImGui.closeCurrentPopup();
         ImGui.endPopup();
      }
      
      ImGui.end();
   }

   public void update(RobotSide side, double linearWrenchMagnitude, double angularWrenchMagnitude)
   {
      sides.get(side).update(linearWrenchMagnitude, angularWrenchMagnitude);
   }

   public void setShowPlots(boolean showPlots)
   {
      for (RobotSide side : RobotSide.values)
      {
         sides.get(side).setShowPlots(showPlots);
      }
   }
}
