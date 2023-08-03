package us.ihmc.rdx.ui.affordances;

import imgui.ImGui;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDX3DPanelHandWrenchIndicator
{
   private final RDX3DPanel panel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final SideDependentList<Double> linearParts = new SideDependentList<>((double) 0, (double) 0);
   private final SideDependentList<Double> angularParts = new SideDependentList<>((double) 0, (double) 0);

   public RDX3DPanelHandWrenchIndicator(RDX3DPanel panel)
   {
      this.panel = panel;
   }

   public void renderImGuiOverlay()
   {
      // temporarily set arbitrary width height
      float panelWidth = 200;
      float panelHeight = 115;

      ImGui.setNextWindowSize(panelWidth, panelHeight);
      float startX = panel.getWindowPositionX() + (panel.getWindowSizeX() - panelWidth - 5);
      float startY = (panel.getWindowPositionY() + 10);
      ImGui.setNextWindowPos(startX, startY);
      ImGui.setNextWindowBgAlpha(0.2f);
      int windowFlags = ImGuiWindowFlags.NoTitleBar; // undecorated
      ImGui.begin(labels.get("WrenchMagnitudeIndicator"), windowFlags);
      ImGui.pushFont(ImGuiTools.getMediumFont());
      for (RobotSide side : RobotSide.values)
      {
         ImGui.text(side.getPascalCaseName() + " Linear: " + String.format("%.2f", linearParts.get(side)) + " N");
         ImGui.text(side.getPascalCaseName() + " Angular: " + String.format("%.2f", angularParts.get(side)) + " Nm");
         if (side == RobotSide.LEFT)
            ImGui.separator();
      }
      ImGui.popFont();
      ImGui.end();
   }

   public void update(RobotSide side, double linearWrenchMagnitude, double angularWrenchMagnitude)
   {
      linearParts.set(side, linearWrenchMagnitude);
      angularParts.set(side, angularWrenchMagnitude);
   }
}
