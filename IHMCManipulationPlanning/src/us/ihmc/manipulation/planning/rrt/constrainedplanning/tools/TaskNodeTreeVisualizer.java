package us.ihmc.manipulation.planning.rrt.constrainedplanning.tools;

import java.awt.Color;

import us.ihmc.commons.PrintTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace.TaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace.TaskNodeTree;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TaskNodeTreeVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final SimulationConstructionSet scs;
   private TaskNodeTree taskNodeTree;
   
   private static boolean showNormalized = true;
   
   public TaskNodeTreeVisualizer(SimulationConstructionSet scs, TaskNodeTree taskNodeTree)
   {
      this.scs = scs;
      this.taskNodeTree = taskNodeTree;
          
   }
   
   public void visualize()
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
            
      /*
       * valid nodes
       */
      for(int i=0;i<taskNodeTree.getWholeNodes().size();i++)
      {
         TaskNode node = taskNodeTree.getWholeNodes().get(i);
         String prefix = taskNodeTree.getTaskName(indexOfDimension)+"_node_"+i;
         if(node.getParentNode() == null)
         {
            PrintTools.info("this is root node");
            yoGraphicsListRegistry.registerArtifact(""+prefix+"_artifact_node", createNode(node, indexOfDimension, prefix, true));
         }
         else
         {  
            yoGraphicsListRegistry.registerArtifact(""+prefix+"_artifact_node", createNode(node, indexOfDimension, prefix, true));
            yoGraphicsListRegistry.registerArtifact(""+prefix+"_artifact_branch", createBranch(node, indexOfDimension, prefix));
         }  
      }
                 
      /*
       * fail nodes
       */
      
      for(int i=0;i<taskNodeTree.getFailNodes().size();i++)
      {
         TaskNode node = taskNodeTree.getFailNodes().get(i);
         String prefix = taskNodeTree.getTaskName(indexOfDimension)+"_fail_node_"+i;
         
         yoGraphicsListRegistry.registerArtifact(""+prefix+"_artifact_node", createNode(node, indexOfDimension, prefix, false));            
      }
            
      simulationOverheadPlotterFactory.setPlotterName("Task Tree Plotter : "+taskNodeTree.getTaskName(indexOfDimension));
      simulationOverheadPlotterFactory.setVariableNameToTrack("");
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.setCreateInSeperateWindow(true);
      simulationOverheadPlotterFactory.createOverheadPlotter();
   }
   
   private YoArtifactLineSegment2d createBranch(TaskNode taskNode, int indexOfDimension, String prefix)
   {        
      YoFrameLineSegment2d yoLine = new YoFrameLineSegment2d(""+prefix+"_line", "", ReferenceFrame.getWorldFrame(), registry);
          
      FramePoint2d nodePoint;
      FramePoint2d parentNodePoint;
      
      if(showNormalized)
      {
         nodePoint = new FramePoint2d(ReferenceFrame.getWorldFrame(), taskNode.getNormalizedNodeData(0), taskNode.getNormalizedNodeData(indexOfDimension), ""+prefix+"_this");
         parentNodePoint = new FramePoint2d(ReferenceFrame.getWorldFrame(), taskNode.getParentNode().getNormalizedNodeData(0), taskNode.getParentNode().getNormalizedNodeData(indexOfDimension), ""+prefix+"_parent");
      }
      else
      {
         nodePoint = new FramePoint2d(ReferenceFrame.getWorldFrame(), taskNode.getNodeData(0), taskNode.getNodeData(indexOfDimension), ""+prefix+"_this");
         parentNodePoint = new FramePoint2d(ReferenceFrame.getWorldFrame(), taskNode.getParentNode().getNodeData(0), taskNode.getParentNode().getNodeData(indexOfDimension), ""+prefix+"_parent");   
      }      
      
      yoLine.set(nodePoint, parentNodePoint);
      
      YoArtifactLineSegment2d artifactLine = new YoArtifactLineSegment2d(""+prefix+"_branch", yoLine, Color.BLACK);
      
      return artifactLine;
   }
   
   private YoArtifactOval createNode(TaskNode taskNode, int indexOfDimension, String prefix, boolean isValid)
   {        
      YoFramePoint yoPoint = new YoFramePoint(""+prefix, ReferenceFrame.getWorldFrame(), registry);
      if(showNormalized)
      {
         yoPoint.setX(taskNode.getNormalizedNodeData(0));
         yoPoint.setY(taskNode.getNormalizedNodeData(indexOfDimension));
      }
      else
      {
         yoPoint.setX(taskNode.getNodeData(0));
         yoPoint.setY(taskNode.getNodeData(indexOfDimension));   
      }
      
      
      YoDouble radius = new YoDouble(""+prefix, registry);
      
      YoArtifactOval artifactOval;
      if(isValid)
      {
         artifactOval = new YoArtifactOval(""+prefix, yoPoint, radius, Color.BLUE);   
      }
      else
      {
         artifactOval = new YoArtifactOval(""+prefix, yoPoint, radius, Color.RED);
      }
      
      radius.set(0.02);   
      
      return artifactOval;
   }
   
}
