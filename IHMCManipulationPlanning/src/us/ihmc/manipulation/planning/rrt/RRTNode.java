package us.ihmc.manipulation.planning.rrt;

import java.util.ArrayList;

/**\
 * RRT Node is a member node of the rapidly randomized tree (RRT).
 * The RRT is the one of the sampling based motion planning tool. 
 * 
 * Every node has node data which of the dimension is larger than 0.
 * 
 * Except root node of the tree, every node has parent node.
 * For the parent node of the 'A' node, 'A' node is a child node of the parent node.
 * This structure excludes extra computing time consuming when <RRTTree> gets a path from root node to goal node.
 *
 * All class that inherits this class <RRTNode> must override two methods, isValid() and createNode().
 * 
 * @param nodeData
 * @param childRRTNode and ParentRRTNode
 * 
 * @author InhoLee 170217
 *
 */
public abstract class RRTNode implements RRTInterface 
{
   private NodeData nodeData;
   private ArrayList<RRTNode> childRRTNode;
   private RRTNode parentRRTNode;

   public RRTNode()
   {

   }

   public RRTNode(RRTNode node)
   {
      this.nodeData = node.nodeData;
      this.childRRTNode = node.childRRTNode;
      this.parentRRTNode = node.parentRRTNode;
   }

   public RRTNode(double[] rootData)
   {
      this.nodeData = new NodeData(rootData.length);
      this.nodeData.q = rootData;
      this.childRRTNode = new ArrayList<RRTNode>();
   }

   public RRTNode(int dimensionOfData)
   {
      this.nodeData = new NodeData(dimensionOfData);
      this.childRRTNode = new ArrayList<RRTNode>();
   }

   public final int getDimensionOfNodeData()
   {
      return nodeData.getDimension();
   }

   public final double getNodeData(int index)
   {
      return nodeData.getQ(index);
   }

   public final double getDistance(RRTNode targetNode)
   {
      return nodeData.distance(targetNode.nodeData);
   }

   public final void setNodeData(int index, double data)
   {
      nodeData.setQ(index, data);
   }

   public final void addChildNode(RRTNode childNode)
   {
      childRRTNode.add(childNode);
      childNode.setParentNode(this);
   }

   public final void setChildNode(int indexOfChild, RRTNode childNode)
   {
      childRRTNode.set(indexOfChild, childNode);
   }

   public final RRTNode getChildNode(int indexOfChild)
   {
      return childRRTNode.get(indexOfChild);
   }

   public final int getNumberOfChild()
   {
      return childRRTNode.size();
   }

   public final void setParentNode(RRTNode parentNode)
   {
      parentRRTNode = parentNode;
   }

   public final RRTNode getParentNode()
   {
      return parentRRTNode;
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

}
