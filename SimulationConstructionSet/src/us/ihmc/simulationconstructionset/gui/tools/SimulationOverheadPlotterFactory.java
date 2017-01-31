package us.ihmc.simulationconstructionset.gui.tools;

import java.awt.Dimension;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;

import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.plotting.PlotterShowHideMenu;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

public class SimulationOverheadPlotterFactory
{
   private static final boolean TRACK_YAW = false;

   private final RequiredFactoryField<SimulationConstructionSet> simulationConstructionSet = new RequiredFactoryField<>("simulationConstructionSet");
   private final RequiredFactoryField<List<YoGraphicsListRegistry>> yoGraphicsListRegistries = new RequiredFactoryField<>("simulationConstructionSet");
   {
      yoGraphicsListRegistries.set(new ArrayList<>());
   }

   private final OptionalFactoryField<Boolean> createInSeperateWindow = new OptionalFactoryField<>("createInSeperateWindow");
   private final OptionalFactoryField<Boolean> showOnStart = new OptionalFactoryField<>("showOnStart");
   private final OptionalFactoryField<String> variableNameToTrack = new OptionalFactoryField<>("variableNameToTrack");

   public SimulationOverheadPlotter createOverheadPlotter()
   {
      FactoryTools.checkAllFactoryFieldsAreSet(this);

      createInSeperateWindow.setDefaultValue(false);
      showOnStart.setDefaultValue(true);

      SimulationOverheadPlotter simulationOverheadPlotter = new SimulationOverheadPlotter();
      simulationOverheadPlotter.setDrawHistory(false);
      simulationOverheadPlotter.setXVariableToTrack(null);
      simulationOverheadPlotter.setYVariableToTrack(null);
      simulationConstructionSet.get().attachPlaybackListener(simulationOverheadPlotter);

      JPanel plotterPanel = simulationOverheadPlotter.getJPanel();
      JPanel plotterKeyJPanel = simulationOverheadPlotter.getJPanelKey();
      JScrollPane scrollPane = new JScrollPane(plotterKeyJPanel);
      String plotterName = "Plotter";
      
      if (createInSeperateWindow.get())
      {
         JFrame overheadWindow = new JFrame(plotterName);
         overheadWindow.setSize(new Dimension(1000, 1000));
         JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT);
         overheadWindow.add(splitPane);


         splitPane.add(plotterPanel);
         splitPane.add(scrollPane);

         splitPane.setResizeWeight(1.0);

         for (int i = 0; i < yoGraphicsListRegistries.get().size(); i++)
         {
            YoGraphicsListRegistry yoGraphicsListRegistry = yoGraphicsListRegistries.get().get(i);
            yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());
            ArrayList<ArtifactList> buffer = new ArrayList<>();
            yoGraphicsListRegistry.getRegisteredArtifactLists(buffer);

            if (variableNameToTrack.hasValue() && !variableNameToTrack.get().isEmpty())
            {
               for (ArtifactList artifactList : buffer)
               {
                  for (Artifact artifact : artifactList.getArtifacts())
                  {
                     if (artifact.getID() == variableNameToTrack.get())
                     {
                        simulationOverheadPlotter.setXVariableToTrack(((YoArtifactPosition) artifact).getYoX());
                        simulationOverheadPlotter.setYVariableToTrack(((YoArtifactPosition) artifact).getYoY());
                     }
                  }
               }
            }
         }

         overheadWindow.setVisible(showOnStart.get());
      }
      else
      {
         simulationConstructionSet.get().addExtraJpanel(plotterPanel, plotterName, showOnStart.get());

         simulationConstructionSet.get().addExtraJpanel(scrollPane, "Plotter Legend", false);

         JScrollPane menuScrollPanel = new JScrollPane(simulationOverheadPlotter.getMenuPanel());
         menuScrollPanel.getVerticalScrollBar().setUnitIncrement(16);
         simulationConstructionSet.get().addExtraJpanel(menuScrollPanel, PlotterShowHideMenu.getPanelName(), false);

         for (int i = 0; i < yoGraphicsListRegistries.get().size(); i++)
         {
            YoGraphicsListRegistry yoGraphicsListRegistry = yoGraphicsListRegistries.get().get(i);
            if (yoGraphicsListRegistry == null)
               continue;

            yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());
         }

         if (variableNameToTrack.hasValue() && !variableNameToTrack.get().isEmpty())
         {
            YoVariable<?> trackingVariable;
            if ((trackingVariable = simulationConstructionSet.get().getVariable(variableNameToTrack + "X")) != null
                  && trackingVariable instanceof DoubleYoVariable)
            {
               simulationOverheadPlotter.setXVariableToTrack((DoubleYoVariable) trackingVariable);
            }
            if ((trackingVariable = simulationConstructionSet.get().getVariable(variableNameToTrack + "Y")) != null
                  && trackingVariable instanceof DoubleYoVariable)
            {
               simulationOverheadPlotter.setYVariableToTrack((DoubleYoVariable) trackingVariable);
            }
            if (TRACK_YAW)
            {
               if ((trackingVariable = simulationConstructionSet.get().getVariable(variableNameToTrack + "Yaw")) != null
                     && trackingVariable instanceof DoubleYoVariable)
               {
                  simulationOverheadPlotter.setYawVariableToTrack((DoubleYoVariable) trackingVariable);
               }
            }
         }
      }

      FactoryTools.disposeFactory(this);
      return simulationOverheadPlotter;
   }

   public void setSimulationConstructionSet(SimulationConstructionSet simulationConstructionSet)
   {
      this.simulationConstructionSet.set(simulationConstructionSet);
   }

   public void addYoGraphicsListRegistries(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.yoGraphicsListRegistries.get().add(yoGraphicsListRegistry);
   }

   public void setCreateInSeperateWindow(boolean createInSeperateWindow)
   {
      this.createInSeperateWindow.set(createInSeperateWindow);
   }
   
   public void setShowOnStart(boolean showOnStart)
   {
      this.showOnStart.set(showOnStart);
   }

   public void setVariableNameToTrack(String variableNameToTrack)
   {
      this.variableNameToTrack.set(variableNameToTrack);
   }
}
