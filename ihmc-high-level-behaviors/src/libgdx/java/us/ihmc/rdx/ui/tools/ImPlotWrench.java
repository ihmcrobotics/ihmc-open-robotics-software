package us.ihmc.rdx.ui.tools;

import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.imgui.ImPlotDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;
import us.ihmc.rdx.imgui.ImPlotPlotPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ImPlotWrench
{
   // wrench
   private SideDependentList<ImPlotPlotPanel> wrenchPlotPanels = new SideDependentList<>();

   private final SideDependentList<ImPlotPlot> linearPlots = new SideDependentList<>();
   private final SideDependentList<ImPlotPlot> angularPlots = new SideDependentList<>();

   private final SideDependentList<ImPlotDoublePlotLine> linearPlotLinesX = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> linearPlotLinesY = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> linearPlotLinesZ = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> angularPlotLinesX = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> angularPlotLinesY = new SideDependentList<>();
   private final SideDependentList<ImPlotDoublePlotLine> angularPlotLinesZ = new SideDependentList<>();

   public ImPlotWrench(RDXBaseUI baseUI)
   {
      for (RobotSide side : RobotSide.values)
      {
         wrenchPlotPanels.set(side, new ImPlotPlotPanel(side.getPascalCaseName() + " Wrench"));
         linearPlotLinesX.set(side, new ImPlotDoublePlotLine(side.getPascalCaseName() + " Wrench Linear: x"));
         linearPlotLinesY.set(side, new ImPlotDoublePlotLine(side.getPascalCaseName() + " Wrench Linear: y"));
         linearPlotLinesZ.set(side, new ImPlotDoublePlotLine(side.getPascalCaseName() + " Wrench Linear: z"));
         angularPlotLinesX.set(side, new ImPlotDoublePlotLine(side.getPascalCaseName() + " Wrench Angular: x"));
         angularPlotLinesY.set(side, new ImPlotDoublePlotLine(side.getPascalCaseName() + " Wrench Angular: y"));
         angularPlotLinesZ.set(side, new ImPlotDoublePlotLine(side.getPascalCaseName() + " Wrench Angular: z"));
         linearPlots.set(side, new ImPlotPlot());
         angularPlots.set(side, new ImPlotPlot());

         linearPlots.get(side).getPlotLines().add(linearPlotLinesX.get(side));
         linearPlots.get(side).getPlotLines().add(linearPlotLinesY.get(side));
         linearPlots.get(side).getPlotLines().add(linearPlotLinesZ.get(side));
         angularPlots.get(side).getPlotLines().add(angularPlotLinesX.get(side));
         angularPlots.get(side).getPlotLines().add(angularPlotLinesY.get(side));
         angularPlots.get(side).getPlotLines().add(angularPlotLinesZ.get(side));

         wrenchPlotPanels.get(side).getPlots().add(linearPlots.get(side));
         wrenchPlotPanels.get(side).getPlots().add(angularPlots.get(side));

         baseUI.getImGuiPanelManager().addPanel(wrenchPlotPanels.get(side));
      }
   }

   public void update(RobotSide side, SpatialVectorReadOnly wrench)
   {
      linearPlotLinesX.get(side).addValue(wrench.getLinearPartX());
      linearPlotLinesY.get(side).addValue(wrench.getLinearPartY());
      linearPlotLinesZ.get(side).addValue(wrench.getLinearPartZ());
      angularPlotLinesX.get(side).addValue(wrench.getAngularPartX());
      angularPlotLinesY.get(side).addValue(wrench.getAngularPartX());
      angularPlotLinesZ.get(side).addValue(wrench.getAngularPartX());
   }
}
