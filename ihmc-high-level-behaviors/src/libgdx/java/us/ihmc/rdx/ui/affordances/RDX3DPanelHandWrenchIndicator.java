package us.ihmc.rdx.ui.affordances;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
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
