package us.ihmc.manipulation.planning.rrt;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.tools.io.printing.PrintTools;

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
   protected ArrayList<RRTNode> pathNode = new ArrayList<RRTNode>();

   private RRTNode nodeCreator;
   
   // numberOfNodes, ArrayList<RRTNode> nodes. every node of the nodes has its parent node.

   public RRTTree(RRTNode rootNode)
   {
      this.rootNode = rootNode;
      nodeCreator = rootNode.createNode();
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
      RRTNode[] nodeOnCurrentLevel;
      RRTNode[] nodeOnFutureLevel;
      int currentLevel = 0;
      int numberOfCurrentLevel = 0;
      int numberOfFutureLevel = 0;
      int numberOfSearch = 0;

      RRTNode optNode = this.rootNode;
      RRTNode curNode = this.rootNode;

      double optMatric = Double.MAX_VALUE;
      double curMatric = getMatric(targetNode, curNode);

      if (curMatric < optMatric)
      {
         optMatric = curMatric;
         optNode = curNode;
      }

      numberOfCurrentLevel++;
      nodeOnCurrentLevel = new RRTNode[numberOfCurrentLevel];
      nodeOnCurrentLevel[currentLevel] = this.rootNode;

      while (true)
      {
         numberOfFutureLevel = 0;
         for (int i = 0; i < numberOfCurrentLevel; i++)
         {
            numberOfFutureLevel = numberOfFutureLevel + nodeOnCurrentLevel[i].getNumberOfChild();
         }

         if (numberOfFutureLevel == 0)
         {
            break;
         }

         int tempNumber = 0;
         nodeOnFutureLevel = new RRTNode[numberOfFutureLevel];
         for (int i = 0; i < numberOfCurrentLevel; i++)
         {
            for (int j = 0; j < nodeOnCurrentLevel[i].getNumberOfChild(); j++)
            {
               nodeOnFutureLevel[tempNumber + j] = nodeOnCurrentLevel[i].getChildNode(j);
               curNode = nodeOnFutureLevel[tempNumber + j];

               curMatric = getMatric(targetNode, curNode);
               if (curMatric < optMatric)
               {
                  optMatric = curMatric;
                  optNode = curNode;
               }
               numberOfSearch++;
            }
            tempNumber = tempNumber + nodeOnCurrentLevel[i].getNumberOfChild();
         }

         numberOfCurrentLevel = numberOfFutureLevel;
         nodeOnCurrentLevel = new RRTNode[numberOfCurrentLevel];
         for (int i = 0; i < numberOfCurrentLevel; i++)
         {
            nodeOnCurrentLevel[i] = nodeOnFutureLevel[i];
         }

         currentLevel++;
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
         RRTPiecewiseNode nodeConnectionTest = new RRTPiecewiseNode(this.nearNode, this.newNode);
         if (nodeConnectionTest.isValidConnection())
         {
            nearNode.addChildNode(this.newNode);
            return true;
         }
      }
      else
      {
         // PrintTools.info("The newly created node is unvalid");
      }
      return false;
   }

   public void updatePath(RRTNode endNode)
   {
      this.pathNode.clear();

      ArrayList<RRTNode> pathNodeOne = new ArrayList<RRTNode>();
      RRTNode singleNode = endNode;
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
         pathNode.add(pathNodeOne.get(pathSize - i - 1));
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

   public ArrayList<RRTNode> getPathNode()
   {
      return pathNode;
   }
}
