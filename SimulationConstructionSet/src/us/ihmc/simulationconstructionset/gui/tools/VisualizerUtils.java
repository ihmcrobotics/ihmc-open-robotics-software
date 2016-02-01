package us.ihmc.simulationconstructionset.gui.tools;

import java.awt.Container;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;

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
   
   public static SimulationOverheadPlotter createOverheadPlotterInSeparateWindow(SimulationConstructionSet scs, boolean showOverheadView, String variableNameToTrack, YoGraphicsListRegistry... yoGraphicsListRegistries)
   {
      JFrame overheadWindow = new JFrame("plotter");
      overheadWindow.setSize(new Dimension(1000, 1000));
      
      SimulationOverheadPlotter plotter = new SimulationOverheadPlotter();
      plotter.setDrawHistory(false);
      plotter.setXVariableToTrack(null);
      plotter.setYVariableToTrack(null);

      scs.attachPlaybackListener(plotter);
      
      JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT);
      overheadWindow.add(splitPane);
      
      JPanel plotterPanel = plotter.getJPanel();
      JPanel plotterKeyJPanel = plotter.getJPanelKey();
      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);

      splitPane.add(plotterPanel);
      splitPane.add(scrollPane);
      
      splitPane.setResizeWeight(1.0);
      
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

      overheadWindow.setVisible(showOverheadView);
      
      return plotter;   
   }
   
}
