package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;

public abstract class TaskNode implements TaskNodeInterface
{
   private NodeData nodeData;
   private NodeData normalizedNodeData;
   private ArrayList<TaskNode> childNodes;
   private TaskNode parentNode;
      
   protected boolean isValid = true;  
   
   public TaskNode()
   {

   }

   public TaskNode(TaskNode node)
   {
      this.nodeData = node.nodeData;
      this.childNodes = node.childNodes;
      this.parentNode = node.parentNode;
      this.normalizedNodeData = node.normalizedNodeData;
   }

   public TaskNode(double[] rootData)
   {
      this.nodeData = new NodeData(rootData.length);
      this.nodeData.q = rootData;
      this.childNodes = new ArrayList<TaskNode>();
      this.normalizedNodeData = new NodeData(rootData.length);
   }

   public TaskNode(int dimensionOfData)
   {
      this.nodeData = new NodeData(dimensionOfData);
      this.childNodes = new ArrayList<TaskNode>();
      this.normalizedNodeData = new NodeData(dimensionOfData);
   }

   public final int getDimensionOfNodeData()
   {
      return nodeData.getDimension();
   }

   public final double getNodeData(int index)
   {
      return nodeData.getQ(index);
   }
   
   public final double getNormalizedNodeData(int index)
   {
      return normalizedNodeData.getQ(index);
   }

   public final double getDistance(TaskNode targetNode)
   {
      return nodeData.distance(targetNode.nodeData);
   }

   public final void setNodeData(int index, double data)
   {
      nodeData.setQ(index, data);
   }
   
   public final void setNormalizedNodeData(int index, double data)
   {
      normalizedNodeData.setQ(index, data);
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
   
   public void setIsValidNode(boolean value)
   {
      isValid = value;
   }
   
   public void convertDataToNormalizedData(TaskNodeRegion nodeRegion)
   {
      normalizedNodeData = new NodeData(getDimensionOfNodeData());
      for(int i=0;i<getDimensionOfNodeData();i++)
      {
         double normalizedValue = 0;
         if(i==0)
         {
            normalizedValue = (getNodeData(i) - nodeRegion.getLowerLimit(i)) / nodeRegion.getTrajectoryTime();
         }
         else
         {
            normalizedValue = (getNodeData(i) - nodeRegion.getLowerLimit(i)) / nodeRegion.sizeOfRegion(i);               
         }
         normalizedNodeData.setQ(i, normalizedValue);
      }
   }
   
   public void convertNormalizedDataToData(TaskNodeRegion nodeRegion)
   {
      nodeData = new NodeData(getDimensionOfNodeData());
      for(int i=0;i<getDimensionOfNodeData();i++)
      {
         double value = 0;
         if(i==0)
         {
            value = getNormalizedNodeData(i)*nodeRegion.getTrajectoryTime() + nodeRegion.getLowerLimit(i);
         }
         else
         {
            value = (getNormalizedNodeData(i)+0.5)*nodeRegion.sizeOfRegion(i) + nodeRegion.getLowerLimit(i);         
         }
         nodeData.setQ(i, value);
      }
   }
   
   @Override
   public abstract boolean isValidNode();

   @Override
   public abstract TaskNode createNode();
}
