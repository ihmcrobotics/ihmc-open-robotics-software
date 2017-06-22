package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;

public class TaskNode
{
   private NodeData nodeData;
   private ArrayList<TaskNode> childNodes;
   private TaskNode parentNode;
      
   public TaskNode()
   {

   }

   public TaskNode(TaskNode node)
   {
      this.nodeData = node.nodeData;
      this.childNodes = node.childNodes;
      this.parentNode = node.parentNode;
   }

   public TaskNode(double[] rootData)
   {
      this.nodeData = new NodeData(rootData.length);
      this.nodeData.q = rootData;
      this.childNodes = new ArrayList<TaskNode>();
   }

   public TaskNode(int dimensionOfData)
   {
      this.nodeData = new NodeData(dimensionOfData);
      this.childNodes = new ArrayList<TaskNode>();
   }

   public final int getDimensionOfNodeData()
   {
      return nodeData.getDimension();
   }

   public final double getNodeData(int index)
   {
      return nodeData.getQ(index);
   }

   public final double getDistance(TaskNode targetNode)
   {
      return nodeData.distance(targetNode.nodeData);
   }

   public final void setNodeData(int index, double data)
   {
      nodeData.setQ(index, data);
   }
   
   public final void setNodeData(TaskNode copyNode)
   {
      for(int i=0;i<copyNode.getDimensionOfNodeData();i++)
      {
         nodeData.setQ(i, copyNode.getNodeData(i));
      }
   }

   public final void addChildNode(TaskNode node)
   {
      childNodes.add(node);
      node.setParentNode(this);
   }

   public final TaskNode getChildNode(int indexOfChild)
   {
      return childNodes.get(indexOfChild);
   }

   public final int getNumberOfChild()
   {
      return childNodes.size();
   }

   public final void setParentNode(TaskNode node)
   {
      parentNode = node;
   }

   public final TaskNode getParentNode()
   {
      return parentNode;
   }
   
   public void printNodeData()
   {
      for(int i=0;i<getDimensionOfNodeData();i++)
         PrintTools.info("" + i +" "+getNodeData(i));
   }
   
   public double getTime()
   {
      return getNodeData(0);
   }
   
   public boolean isValidNode()
   {
      return true;
   }
}
