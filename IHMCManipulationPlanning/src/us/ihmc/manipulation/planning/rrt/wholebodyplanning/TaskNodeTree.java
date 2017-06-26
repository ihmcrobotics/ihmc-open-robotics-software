package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import us.ihmc.commons.PrintTools;

public class TaskNodeTree
{      
   private TaskNode rootNode;
   private TaskNode nearNode;
   private TaskNode newNode;
   private TaskNode randomNode;
   
   private ArrayList<TaskNode> path = new ArrayList<TaskNode>();
   
   private ArrayList<TaskNode> wholeNodes = new ArrayList<TaskNode>();
   private ArrayList<TaskNode> failNodes = new ArrayList<TaskNode>();
   
   private TaskNodeRegion nodeRegion;
   
   /*
    * If @param matricRatioTimeToTask is 0.3, the matric will be obtained as much as (getTimeGap*0.3 + getTaskDisplacement*0.7).
    */   
   private double matricRatioTimeToTask = 0.1;
   
   private double maximumDisplacementOfStep = 0.3;
   private double maximumTimeGapOfStep = 0.3;
   
   private int dimensionOfTask;

   private ArrayList<String> taskNames;
   
   private double trajectoryTime;
   
   public TaskNodeTree(TaskNode rootNode)
   {
      this.rootNode = rootNode;
      this.wholeNodes.add(this.rootNode);
      
      this.nodeRegion = new TaskNodeRegion(this.rootNode.getDimensionOfNodeData());
      
      this.dimensionOfTask = rootNode.getDimensionOfNodeData()-1;
      
      this.taskNames = new ArrayList<String>();
      this.taskNames.add("time");
      for(int i=1;i<this.dimensionOfTask+1;i++)
         this.taskNames.add("Task_"+i+"_"+"..");
   }
   
   public TaskNodeTree(TaskNode rootNode, String... taskNames)
   {
      this.rootNode = rootNode;
      this.wholeNodes.add(this.rootNode);
      
      this.nodeRegion = new TaskNodeRegion(this.rootNode.getDimensionOfNodeData());
      
      this.dimensionOfTask = rootNode.getDimensionOfNodeData()-1;
      
      this.taskNames = new ArrayList<String>();
      this.taskNames.add("time");
      if(this.dimensionOfTask != taskNames.length)
         PrintTools.warn("Task dimension is incorrect");
      else
         for(int i=1;i<this.dimensionOfTask+1;i++)
            this.taskNames.add("Task_"+i+"_"+taskNames[i-1]);
   }
      
   public String getTaskName(int indexOfDimension)
   {
      return taskNames.get(indexOfDimension);
   }   
   
   public int getDimensionOfTask()
   {
      return dimensionOfTask;
   }
      
   public double getTrajectoryTime()
   {
      trajectoryTime = nodeRegion.getTrajectoryTime();
      return trajectoryTime;
   }
      
   private void setRandomNodeData(TaskNode node, int index)
   {
      Random randomManager = new Random();
      double value = randomManager.nextDouble() * (nodeRegion.getUpperLimit(index) - nodeRegion.getLowerLimit(index)) + nodeRegion.getLowerLimit(index);
      node.setNodeData(index, value);
   }
    
   private void setRandomNodeData(TaskNode node)
   {
      for(int i=0;i<node.getDimensionOfNodeData();i++)
         setRandomNodeData(node, i);
   }
      
   public void setMatricRatioTimeToTask(double ratio)
   {
      matricRatioTimeToTask = ratio;
   }
   
   public void expandTree(int numberOfExpanding)
   {
      for(int i=0;i<numberOfExpanding;i++)
      {
         PrintTools.info("expanding process "+i);         
         expandingTree();
      }
   }
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   private double getTaskDisplacement(TaskNode nodeOne, TaskNode nodeTwo)
   {
      double squaredDisplacement = 0;
      
      for(int i=1;i<rootNode.getDimensionOfNodeData();i++)
      {
         double nodeOneValue = nodeOne.getNodeData(i);
         double nodeTwoValue = nodeTwo.getNodeData(i);
         squaredDisplacement = (nodeOneValue - nodeTwoValue) * (nodeOneValue - nodeTwoValue);
      }      
      
      return Math.sqrt(squaredDisplacement);
   }
   
   private double getTimeGap(TaskNode nodeOld, TaskNode nodeNew)
   {
      return nodeNew.getTime() - nodeOld.getNodeData(0);
   }
   
   private double getMatric(TaskNode nodeOne, TaskNode nodeTwo)
   {      
      double timeGap = getTimeGap(nodeOne, nodeTwo);
      
      if(timeGap <= 0)
         return Double.MAX_VALUE;
      else
      {
         double matric = 0;
         matric = timeGap * matricRatioTimeToTask + getTaskDisplacement(nodeOne, nodeTwo) * (1-matricRatioTimeToTask);
         return matric;
      }
   }
   
   private TaskNode createNode()
   {
      return rootNode.createNode();
   }
   
   private void expandingTree()
   {
      updateRandomConfiguration();
      updateNearestNode();
      updateNewConfiguration();
      connectNewConfiguration();
   }
   
   private void updateRandomConfiguration()
   {
      TaskNode randomNode = createNode();
      setRandomNodeData(randomNode);
      this.randomNode = randomNode;
   }
   
   private void updateNearestNode()
   {
      TaskNode nearNode = this.rootNode;
      TaskNode curNode;
      
      double optMatric = Double.MAX_VALUE;
      double curMatric;
            
      for(int i=0;i<wholeNodes.size();i++)
      {
         curNode = this.wholeNodes.get(i);
         curMatric = getMatric(curNode, randomNode);
         if (curMatric < optMatric)
         {
            optMatric = curMatric;
            nearNode = curNode;
         }   
      }
      
      this.nearNode = nearNode;
   }
   
   private void updateNewConfiguration()
   {
      TaskNode newNode = createNode();
      
      double timeGap = getTimeGap(this.nearNode, this.randomNode);
      double displacement = getTaskDisplacement(this.nearNode, this.randomNode); 
      
      double expandingTimeGap;
      double expandingDisplacement;
      
      // timeGap Clamping
      if(timeGap > maximumTimeGapOfStep)
      {
         expandingTimeGap = maximumTimeGapOfStep;
      }
      else
      {
         expandingTimeGap = timeGap;         
      }      
      expandingDisplacement = displacement*expandingTimeGap/timeGap;
      
      // displacement Clamping
      if(expandingDisplacement > maximumDisplacementOfStep)
      {
         expandingDisplacement = maximumDisplacementOfStep;
         expandingTimeGap = timeGap*maximumDisplacementOfStep/displacement;
      }
      
      // set
      newNode.setNodeData(0, nearNode.getTime() + expandingTimeGap);
      for(int i=1;i<newNode.getDimensionOfNodeData();i++)
      {
         double iDisplacement = (this.randomNode.getNodeData(i) - nearNode.getNodeData(i))/displacement*expandingDisplacement;
         newNode.setNodeData(i, nearNode.getNodeData(i) + iDisplacement);
         //PrintTools.info("expandingDisplacement "+expandingTimeGap + " " + expandingDisplacement + " " + iDisplacement);
      }     
      
      for (int i = 0; i < this.randomNode.getDimensionOfNodeData(); i++)
      {
         //PrintTools.info("targetNode "+targetNode.getNodeData(i) + " ");
      }
      for (int i = 0; i < nearNode.getDimensionOfNodeData(); i++)
      {
         //PrintTools.info("nearNode "+nearNode.getNodeData(i) + " ");
      }
      for (int i = 0; i < newNode.getDimensionOfNodeData(); i++)
      {
         //PrintTools.info("newNode "+newNode.getNodeData(i) + " ");
      }
      
      this.newNode = newNode;
   }
   
   private void connectNewConfiguration()
   {
      if(this.newNode.isValidNode())
      {
         nearNode.addChildNode(this.newNode);
         wholeNodes.add(newNode);
         PrintTools.info("this new Configuration is added on tree");         
      }
      else
      {
         failNodes.add(this.newNode);
         PrintTools.info("this new Configuration cannot be added on tree");
      }
   }
   
   public TaskNodeRegion getTaskNodeRegion()
   {
      return nodeRegion;
   }

   public ArrayList<TaskNode> getPath()
   {
      return path;
   }
   
   public ArrayList<TaskNode> getWholeNodes()
   {
      return wholeNodes;
   }
   
   public ArrayList<TaskNode> getFailNodes()
   {
      return failNodes;
   }
   
   public void saveNodes()
   {
      String fileName = "/home/shadylady/tree.txt";
      BufferedWriter bw = null;
      FileWriter fw = null;
      
      try {
         String savingContent = "";
         
         for(int i=0;i<getWholeNodes().size();i++)
         {
            String convertedNodeData = "";            
            
            convertedNodeData = convertedNodeData + "1\t";
            for(int j=0;j<getWholeNodes().get(i).getDimensionOfNodeData();j++)
            {
               convertedNodeData = convertedNodeData + String.format("%.3f\t", getWholeNodes().get(i).getNodeData(j));               
            }
            
            if(getWholeNodes().get(i).getParentNode() == null)
            {
               for(int j=0;j<getWholeNodes().get(i).getDimensionOfNodeData();j++)
               {
                  convertedNodeData = convertedNodeData + "0\t";               
               }
            }
            else
            {
               for(int j=0;j<getWholeNodes().get(i).getDimensionOfNodeData();j++)
               {
                  convertedNodeData = convertedNodeData + String.format("%.3f\t", getWholeNodes().get(i).getParentNode().getNodeData(j));               
               }   
            }
            convertedNodeData = convertedNodeData + "\n";
            
            savingContent = savingContent + convertedNodeData;
         }
         
         for(int i=0;i<getFailNodes().size();i++)
         {
            String convertedNodeData = "";            
            
            convertedNodeData = convertedNodeData + "2\t";
            for(int j=0;j<getFailNodes().get(i).getDimensionOfNodeData();j++)
            {
               convertedNodeData = convertedNodeData + String.format("%.3f\t", getFailNodes().get(i).getNodeData(j));               
            }
            convertedNodeData = convertedNodeData + "\n";
            
            savingContent = savingContent + convertedNodeData;
         }
         
         
         fw = new FileWriter(fileName);
         bw = new BufferedWriter(fw);
         bw.write(savingContent);

         System.out.println("Save Done");

      } catch (IOException e) {

         e.printStackTrace();

      } finally {

         try {

            if (bw != null)
               bw.close();

            if (fw != null)
               fw.close();

         } catch (IOException ex) {

            ex.printStackTrace();

         }

      }
   }
}
