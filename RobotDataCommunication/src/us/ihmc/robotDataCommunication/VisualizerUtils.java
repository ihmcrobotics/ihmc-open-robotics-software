package us.ihmc.robotDataCommunication;

import javax.swing.JPanel;
import javax.swing.JScrollPane;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.plotting.SimulationOverheadPlotter;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class VisualizerUtils
{

   public static void createOverheadPlotter(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, SimulationConstructionSet scs, boolean showOverheadView)
   {
      SimulationOverheadPlotter plotter = new SimulationOverheadPlotter();
      plotter.setDrawHistory(false);
      plotter.setXVariableToTrack(null);
      plotter.setYVariableToTrack(null);
   
      scs.attachPlaybackListener(plotter);
      JPanel plotterPanel = plotter.getJPanel();
      String plotterName = "Plotter";
      scs.addExtraJpanel(plotterPanel, plotterName);
      JPanel plotterKeyJPanel = plotter.getJPanelKey();
   
      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);
   
      scs.addExtraJpanel(scrollPane, "Plotter Legend");
   
      dynamicGraphicObjectsListRegistry.addArtifactListsToPlotter(plotter.getPlotter());
      
      if (showOverheadView)
         scs.getStandardSimulationGUI().selectPanel(plotterName);
   }

}
