package us.ihmc.rdx.ui.affordances;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.yo.ImPlotDoublePlotLine;
import us.ihmc.rdx.ui.yo.ImPlotPlot;
import us.ihmc.rdx.ui.yo.ImPlotPlotLine;
import us.ihmc.rdx.ui.yo.ImPlotPlotPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class ImArmTorquePlot
{
   // actuator torques: arm has 7 joint torques
   private SideDependentList<ImPlotPlotPanel> torquePlotPanels = new SideDependentList<>();
   private SideDependentList<ImPlotPlot> torquePlots = new SideDependentList<>();
   private final SideDependentList<ArrayList<ImPlotDoublePlotLine>> plot_lines = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());

   public ImArmTorquePlot(RDXBaseUI baseUI, String title, SideDependentList<List<OneDoFJointBasics>> armJoints)
   {
      for (RobotSide side : RobotSide.values)
      {
         torquePlotPanels.set(side, new ImPlotPlotPanel(side + ": " + title));
         torquePlots.set(side, new ImPlotPlot());
         List<OneDoFJointBasics> oneArmJoints = armJoints.get(side);
         for (OneDoFJointBasics joint : oneArmJoints)
         {
            ImPlotDoublePlotLine doublePlotLine = new ImPlotDoublePlotLine(joint.getName() + " torque");
            plot_lines.get(side).add(doublePlotLine);
            torquePlots.get(side).getPlotLines().add(doublePlotLine);
         }
         torquePlotPanels.get(side).getPlots().add(torquePlots.get(side));

         baseUI.getImGuiPanelManager().addPanel(torquePlotPanels.get(side));
      }
   }

   public void update(RobotSide side, double[] jointTorques)
   {
      for (int i = 0; i < jointTorques.length; ++i)
      {
         plot_lines.get(side).get(i).addValue(jointTorques[i]);
      }
   }
}


