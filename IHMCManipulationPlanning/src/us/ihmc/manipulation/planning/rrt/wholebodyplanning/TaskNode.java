package us.ihmc.manipulation.planning.rrt.wholebodyplanning;

import java.util.ArrayList;

public abstract class TaskNode
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

   public class NodeData
   {
      private double[] q;
      private int dimension;

      private NodeData(int dimension)
      {
         this.q = new double[dimension];
         this.dimension = dimension;
      }

      private NodeData(NodeData nodeData)
      {
         this.q = new double[nodeData.getDimension()];
         for (int i = 0; i < nodeData.getDimension(); i++)
         {
            this.q[i] = nodeData.getQ(i);
         }
         this.dimension = nodeData.getDimension();
      }

      private final int getDimension()
      {
         return dimension;
      }

      private final double getQ(int index)
      {
         return q[index];
      }

      private final void setQ(int index, double value)
      {
         this.q[index] = value;
      }

      private final double distance(NodeData nodeData)
      {
         double ret = 0;
         for (int i = 0; i < dimension; i++)
         {
            ret = ret + (nodeData.getQ(i) - this.getQ(i)) * (nodeData.getQ(i) - this.getQ(i));
         }
         ret = Math.sqrt(ret);

         return ret;
      }
   }
   
   public abstract void setRandomNodeData();
}
