package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;

public class TaskNodeTreeVisualizer
{
   private final SimulationConstructionSet scs;
   private TaskNodeTree taskNodeTree;
   
   public TaskNodeTreeVisualizer(SimulationConstructionSet scs, TaskNodeTree taskNodeTree)
   {
      this.scs = scs;
      this.taskNodeTree = taskNodeTree;
          
      visualize();
   }
   
   private void visualize()
   {
      int dimensionOfTask = taskNodeTree.getDimensionOfTask();
      
      
      for(int i=1;i<dimensionOfTask+1;i++)
      {
         SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
         
         plottingTaskTree(simulationOverheadPlotterFactory, i);
      }
   }
   
   private void plottingTaskTree(SimulationOverheadPlotterFactory simulationOverheadPlotterFactory, int indexOfDimension)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      simulationOverheadPlotterFactory.setPlotterName("Task Tree Plotter : "+taskNodeTree.getTaskName(indexOfDimension));
      simulationOverheadPlotterFactory.setVariableNameToTrack("centroidGraphic");
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.setCreateInSeperateWindow(true);
      simulationOverheadPlotterFactory.createOverheadPlotter();
   }
}
