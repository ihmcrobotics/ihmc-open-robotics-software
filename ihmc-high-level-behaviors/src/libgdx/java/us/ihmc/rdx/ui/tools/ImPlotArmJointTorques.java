package us.ihmc.rdx.ui.tools;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.imgui.ImPlotDoublePlotLine;
import us.ihmc.rdx.imgui.ImPlotPlot;
import us.ihmc.rdx.imgui.ImPlotPlotPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

public class ImPlotArmJointTorques
{
   // actuator torques: arm has 7 joint torques
   private SideDependentList<ImPlotPlotPanel> torquePlotPanels = new SideDependentList<>();
   private SideDependentList<ImPlotPlot> torquePlots = new SideDependentList<>();
   private final SideDependentList<ArrayList<ImPlotDoublePlotLine>> plotLines = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());

   public ImPlotArmJointTorques(RDXBaseUI baseUI, String title, SideDependentList<List<OneDoFJointBasics>> armJoints)
   {
      this(baseUI, title, "", armJoints);
   }

   public ImPlotArmJointTorques(RDXBaseUI baseUI, String title, String prefix, SideDependentList<List<OneDoFJointBasics>> armJoints)
   {
      for (RobotSide side : RobotSide.values)
      {
         torquePlotPanels.set(side, new ImPlotPlotPanel(side.getPascalCaseName() + " " + title));
         torquePlots.set(side, new ImPlotPlot());
         List<OneDoFJointBasics> oneArmJoints = armJoints.get(side);
         for (int i = 0; i < oneArmJoints.size(); ++i)
         {
            ImPlotDoublePlotLine doublePlotLine = new ImPlotDoublePlotLine(oneArmJoints.get(i).getName() + " " + prefix + " torque");
            plotLines.get(side).add(doublePlotLine);
            torquePlots.get(side).getPlotLines().add(doublePlotLine);
         }
         torquePlotPanels.get(side).getPlots().add(torquePlots.get(side));
         baseUI.getImGuiPanelManager().queueAddPanel(torquePlotPanels.get(side));
      }
   }

   public void update(RobotSide side, double[] jointTorques)
   {
      for (int i = 0; i < jointTorques.length; ++i)
      {
         plotLines.get(side).get(i).addValue(jointTorques[i]);
      }
   }
}


