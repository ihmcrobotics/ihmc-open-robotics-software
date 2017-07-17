package us.ihmc.manipulation.planning.rrt.generalrrt;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.commons.PrintTools;

/**
 * Every tree has a root node.
 * When the initiate this RRTTree class, user must define the following parameters for a tree. * 
 * @param root node.
 * @param searching bound for the random exploring expand.
 * @param step length.
 * 
 * This RRT class provides expanding and find a path from selected node to the root node of this tree.
 * 
 * The basic expanding process is 
 * 1. get random node in boundary.
 * 2. from the random node, find near node among the all nodes in tree.
 * 3. create a new node between the random node and the near node. If they are close as much as the step length, the random node become new node.
 * 4. add the new node to tree
 * 
 * <expandTree> find random node with uniform random value in boundary.
 * Voronoi diagram will be added in near future.
 * 
 * @author InhoLee 170217
 *
 */
public class RRTTree
{
   protected RRTNode rootNode;
   protected RRTNode nearNode;
   protected RRTNode newNode;
   protected double stepLength;

   protected RRTNode upperBoundNode;
   protected RRTNode lowerBoundNode;
   protected ArrayList<RRTNode> pathNodes = new ArrayList<RRTNode>();
   
   protected ArrayList<RRTNode> wholeNodes = new ArrayList<RRTNode>();
   public ArrayList<RRTNode> failNodes = new ArrayList<RRTNode>();

   protected RRTNode nodeCreator;
   
   // numberOfNodes, ArrayList<RRTNode> nodes. every node of the nodes has its parent node.

   public RRTTree(RRTNode rootNode)
   {
      this.rootNode = rootNode;
      nodeCreator = rootNode.createNode();
      wholeNodes.add(this.rootNode);
   }

   public void setStepLength(double length)
   {
      this.stepLength = length;
   }

   public double getStepLength()
   {
      return this.stepLength;
   }

   public void setUpperBound(RRTNode upperBoundNode)
   {
      this.upperBoundNode = upperBoundNode;
   }

   public void setLowerBound(RRTNode lowerBoundNode)
   {
      this.lowerBoundNode = lowerBoundNode;
   }

   // User can override
   public double getMatric(RRTNode nodeOne, RRTNode nodeTwo)
   {
      return nodeOne.getDistance(nodeTwo);
   }

   public RRTNode getRandomNode()
   {
      warningMessage();
      RRTNode randomNode = nodeCreator.createNode();
      Random randomManager = new Random();
      for (int i = 0; i < randomNode.getDimensionOfNodeData(); i++)
      {
         double randonValue = randomManager.nextDouble() * (this.upperBoundNode.getNodeData(i) - this.lowerBoundNode.getNodeData(i))
               + this.lowerBoundNode.getNodeData(i);
         randomNode.setNodeData(i, randonValue);
      }
      return randomNode;
   }

   public boolean expandTree()
   {
      RRTNode node = getRandomNode();
      updateNearNodeForTargetNode(node);
      this.newNode = getNewNode(node);

      return addNewNode();
   }

   public boolean expandTree(RRTNode node)
   {
      updateNearNodeForTargetNode(node);
      this.newNode = getNewNode(node);
      return addNewNode();
   }

   public void updateNearNodeForTargetNode(RRTNode targetNode)
   {
      RRTNode optNode = this.wholeNodes.get(0);
      RRTNode curNode;

      double optMatric = Double.MAX_VALUE;
      double curMatric;
      
      for(int i=0;i<this.wholeNodes.size();i++)
      {
         curNode = this.wholeNodes.get(i);
         curMatric = getMatric(curNode, targetNode);
         if (curMatric < optMatric)
         {
            optMatric = curMatric;
            optNode = curNode;
         }         
      }
      
      this.nearNode = optNode;
   }
   

   public RRTNode getNewNode(RRTNode targetNode)
   {
      RRTNode newNode = nodeCreator.createNode();
      if (targetNode.getDistance(nearNode) < getStepLength())
      {
         newNode = targetNode;
      }
      else
      {
         for (int i = 0; i < targetNode.getDimensionOfNodeData(); i++)
         {
            double normalizedDistance = (targetNode.getNodeData(i) - nearNode.getNodeData(i)) / nearNode.getDistance(targetNode);
            newNode.setNodeData(i, nearNode.getNodeData(i) + getStepLength() * normalizedDistance);
         }
      }
      return newNode;
   }

   public boolean addNewNode()
   {
      if (this.newNode.isValidNode() == true)
      {
         RRTValidConnection rrtValidConnection = new RRTValidConnection(this.nearNode, this.newNode);
         if (rrtValidConnection.isValidConnection())
         {
            nearNode.addChildNode(this.newNode);
            wholeNodes.add(newNode);
            return true;
         }
      }
      else
      {
         failNodes.add(newNode);
      }
      return false;
   }

   public void updatePathNode(RRTNode endNodeOfPath)
   {
      this.pathNodes.clear();

      ArrayList<RRTNode> pathNodeOne = new ArrayList<RRTNode>();
      RRTNode singleNode = endNodeOfPath;
      while (true)
      {
         pathNodeOne.add(singleNode);
         if (singleNode == rootNode)
         {
            PrintTools.info("node Path is completely built");
            break;
         }
         singleNode = singleNode.getParentNode();
      }

      int pathSize = pathNodeOne.size();
      for (int i = 0; i < pathSize; i++)
      {
         pathNodes.add(pathNodeOne.get(pathSize - i - 1));
      }
   }

   private void warningMessage()
   {
      if (upperBoundNode == null || lowerBoundNode == null)
      {
         PrintTools.info("Searching boundary should be defined (Use methods setUpperBound and setLowerBound)");
      }
      if (stepLength == 0)
      {
         PrintTools.info("Step length should be defined (Use methods setStepLength)");
      }
   }

   public RRTNode getRootNode()
   {
      return rootNode;
   }

   public RRTNode getNearNode()
   {
      return nearNode;
   }

   public RRTNode getNewNode()
   {
      return newNode;
   }

   public ArrayList<RRTNode> getPathNodes()
   {
      return pathNodes;
   }
   
   public ArrayList<RRTNode> getWholeNodes()
   {
      return wholeNodes;
   }
   
   public ArrayList<RRTNode> getFailNodes()
   {
      return failNodes;
   }
   
   
}
