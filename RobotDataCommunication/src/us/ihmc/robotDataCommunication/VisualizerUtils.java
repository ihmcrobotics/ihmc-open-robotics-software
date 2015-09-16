package us.ihmc.robotDataCommunication;

import java.util.ArrayList;

import javax.swing.JPanel;
import javax.swing.JScrollPane;

import us.ihmc.plotting.Artifact;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.plotting.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;

public class VisualizerUtils
{
   public static SimulationOverheadPlotter createOverheadPlotter(SimulationConstructionSet scs, boolean showOverheadView, YoGraphicsListRegistry... yoGraphicsListRegistries)
   {
      return createOverheadPlotter(scs, showOverheadView, null, yoGraphicsListRegistries);
   }

   public static SimulationOverheadPlotter createOverheadPlotter(SimulationConstructionSet scs, boolean showOverheadView, String variableNameToTrack, YoGraphicsListRegistry... yoGraphicsListRegistries)
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

      for (int i = 0; i < yoGraphicsListRegistries.length; i++)
      {
         YoGraphicsListRegistry yoGraphicsListRegistry = yoGraphicsListRegistries[i];
         yoGraphicsListRegistry.addArtifactListsToPlotter(plotter.getPlotter());
         ArrayList<ArtifactList> buffer = new ArrayList<>();
         yoGraphicsListRegistry.getRegisteredArtifactLists(buffer);

         if (variableNameToTrack != null && !variableNameToTrack.isEmpty())
         {
            for (ArtifactList artifactList : buffer)
            {
               for (Artifact artifact : artifactList.getArtifacts())
               {
                  if (artifact.getID() == variableNameToTrack)
                  {
                     plotter.setXVariableToTrack(((YoArtifactPosition) artifact).getVariables()[0]);
                     plotter.setYVariableToTrack(((YoArtifactPosition) artifact).getVariables()[1]);
                  }
               }
            }
         }
      }

      if (showOverheadView)
         scs.getStandardSimulationGUI().selectPanel(plotterName);
      
      return plotter;
   }
}
