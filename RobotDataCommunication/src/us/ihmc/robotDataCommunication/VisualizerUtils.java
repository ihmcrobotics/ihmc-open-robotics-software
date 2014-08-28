package us.ihmc.robotDataCommunication;

import java.util.ArrayList;

import javax.swing.JPanel;
import javax.swing.JScrollPane;

import us.ihmc.commonWalkingControlModules.captureRegion.CommonCapturePointCalculator;
import us.ihmc.plotting.Artifact;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPosition;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.plotting.SimulationOverheadPlotter;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class VisualizerUtils
{
   public static void createOverheadPlotter(SimulationConstructionSet scs, boolean showOverheadView, DynamicGraphicObjectsListRegistry... dynamicGraphicObjectsListRegistries)
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

      for (int i = 0; i < dynamicGraphicObjectsListRegistries.length; i++)
      {
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = dynamicGraphicObjectsListRegistries[i];
         dynamicGraphicObjectsListRegistry.addArtifactListsToPlotter(plotter.getPlotter());
         ArrayList<ArtifactList> buffer = new ArrayList<>();
         dynamicGraphicObjectsListRegistry.getRegisteredArtifactLists(buffer);

         for (ArtifactList artifactList : buffer)
         {
            for (Artifact artifact : artifactList.getArtifacts())
            {
               if (artifact.getID() == CommonCapturePointCalculator.CAPTURE_POINT_DYNAMIC_GRAPHIC_OBJECT_NAME)
               {
                  plotter.setXVariableToTrack(((YoArtifactPosition) artifact).getVariables()[0]);
                  plotter.setYVariableToTrack(((YoArtifactPosition) artifact).getVariables()[1]);
               }
            }
         }
      }

      if (showOverheadView)
         scs.getStandardSimulationGUI().selectPanel(plotterName);
   }
}
