package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.flag.ImGuiWindowFlags;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDX3DPanelHandWrenchIndicator
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private boolean showAndUpdate = true;
   private SideDependentList<Double> linearParts = new SideDependentList<>((double) 0, (double) 0);
   private SideDependentList<Double> angularParts = new SideDependentList<>((double) 0, (double) 0);

   public void render(float windowWidth, float windowHeight, float windowPosX, float windowPosY)
   {
      if (showAndUpdate)
      {
         // temporarily set arbitrary width height
         float panelWidth = 200;
         float panelHeight = 110;

         ImGui.setNextWindowSize(panelWidth, panelHeight);
         float startX = windowPosX + (windowWidth - panelWidth - 5);
         float startY = (windowPosY + 10);
         ImGui.setNextWindowPos(startX, startY);
         ImGui.setNextWindowBgAlpha(0.2f);
         int windowFlags = ImGuiWindowFlags.NoTitleBar; // undecorated
         ImGui.begin(labels.get("WrenchMagnitudeIndicator"), windowFlags);
         ImGui.pushFont(ImGuiTools.getMediumFont());
         for (RobotSide side : RobotSide.values)
         {
            ImGui.text(side.getPascalCaseName() + " Linear: " + String.format("%.2f", linearParts.get(side)) + " N");
            ImGui.text(side.getPascalCaseName() + " Angular: " + String.format("%.2f", angularParts.get(side)) + " Nm");
            ImGui.separator();
         }
         ImGui.popFont();
         ImGui.end();
      }
   }

   public void update(RobotSide side, double linearWrenchMagnitude, double angularWrenchMagnitude)
   {
      if (showAndUpdate)
      {
         linearParts.set(side, linearWrenchMagnitude);
         angularParts.set(side, angularWrenchMagnitude);
      }
   }

   public void setShowAndUpdate(boolean showAndUpdate)
   {
      this.showAndUpdate = showAndUpdate;
   }
}
