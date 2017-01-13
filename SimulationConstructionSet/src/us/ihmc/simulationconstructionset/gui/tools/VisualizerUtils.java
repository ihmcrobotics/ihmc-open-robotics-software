package us.ihmc.simulationconstructionset.gui.tools;

import java.awt.Dimension;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;

import us.ihmc.graphics3DDescription.plotting.artifact.Artifact;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphics3DDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphics3DDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.plotting.PlotterShowHideMenu;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;

public class VisualizerUtils
{
   private static final boolean TRACK_YAW = false;
   
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

      JScrollPane menuScrollPanel = new JScrollPane(plotter.getMenuPanel());
      menuScrollPanel.getVerticalScrollBar().setUnitIncrement(16);
      scs.addExtraJpanel(menuScrollPanel, PlotterShowHideMenu.getPanelName());

      for (int i = 0; i < yoGraphicsListRegistries.length; i++)
      {
         YoGraphicsListRegistry yoGraphicsListRegistry = yoGraphicsListRegistries[i];
         if (yoGraphicsListRegistry == null)
            continue;

         yoGraphicsListRegistry.addArtifactListsToPlotter(plotter.getPlotter());
      }
      
      if (variableNameToTrack != null && !variableNameToTrack.isEmpty())
      {
         YoVariable<?> trackingVariable;
         if ((trackingVariable = scs.getVariable(variableNameToTrack + "X")) != null && trackingVariable instanceof DoubleYoVariable)
         {
            plotter.setXVariableToTrack((DoubleYoVariable) trackingVariable);
         }
         if ((trackingVariable = scs.getVariable(variableNameToTrack + "Y")) != null && trackingVariable instanceof DoubleYoVariable)
         {
            plotter.setYVariableToTrack((DoubleYoVariable) trackingVariable);
         }
         if (TRACK_YAW)
         {
            if ((trackingVariable = scs.getVariable(variableNameToTrack + "Yaw")) != null && trackingVariable instanceof DoubleYoVariable)
            {
               plotter.setYawVariableToTrack((DoubleYoVariable) trackingVariable);
            }
         }
      }

      if (showOverheadView)
      {
         scs.getStandardSimulationGUI().selectPanel(plotterName);
      }

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
                     plotter.setXVariableToTrack(((YoArtifactPosition) artifact).getYoX());
                     plotter.setYVariableToTrack(((YoArtifactPosition) artifact).getYoY());
                  }
               }
            }
         }
      }

      overheadWindow.setVisible(showOverheadView);

      return plotter;
   }

}
