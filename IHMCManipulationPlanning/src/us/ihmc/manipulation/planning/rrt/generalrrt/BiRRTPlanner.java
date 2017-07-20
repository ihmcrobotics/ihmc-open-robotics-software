package us.ihmc.manipulation.planning.rrt.generalrrt;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;

public class BiRRTPlanner
{
   private RRTNode rootNode;
   private RRTNode goalNode;

   public RRTTree rrtTreeRoot;
   public RRTTree rrtTreeGoal;
   public RRTPiecewisePath rrtPiecewisePath;
   public ArrayList<RRTNode> pathNodes = new ArrayList<RRTNode>();
   public ArrayList<RRTNode> optimalPath = new ArrayList<RRTNode>();

   public BiRRTPlanner(RRTNode root, RRTNode goal, double stepLength)
   {
      this.rootNode = root;
      this.goalNode = goal;

      rrtTreeRoot = new RRTTree(this.rootNode);
      rrtTreeRoot.setStepLength(stepLength);

      rrtTreeGoal = new RRTTree(this.goalNode);
      rrtTreeGoal.setStepLength(stepLength);
   }

   public void setBound(RRTNode lowerBoundNode, RRTNode upperBoundNode)
   {
      rrtTreeRoot.setLowerBound(lowerBoundNode);
      rrtTreeRoot.setUpperBound(upperBoundNode);

      rrtTreeGoal.setLowerBound(lowerBoundNode);
      rrtTreeGoal.setUpperBound(upperBoundNode);
   }

   public boolean expandTreeGoal(int maxNumberOfExpanding)
   {
      for (int i = 0; i < maxNumberOfExpanding; i++)
      {
         RRTTree treeToExpand;
         RRTTree treeNotToExpand;

         if (i % 2 == 0)
         {
            treeToExpand = rrtTreeRoot;
            treeNotToExpand = rrtTreeGoal;
         }
         else
         {
            treeToExpand = rrtTreeGoal;
            treeNotToExpand = rrtTreeRoot;
         }

         if (treeToExpand.expandTree() == true)
         {
            RRTNode jointNode = getClosestNode(treeToExpand.newNode, treeNotToExpand);
            if (jointNode.getDistance(treeToExpand.newNode) < treeToExpand.getStepLength())
            {
               // valid reach?
               RRTValidConnection rrtValidConnection = new RRTValidConnection(jointNode, treeToExpand.newNode);
               if (rrtValidConnection.isValidConnection())
               {
                  treeToExpand.updatePathNode(treeToExpand.newNode);
                  treeNotToExpand.updatePathNode(jointNode);

                  optimalPath.clear();
                  if (i % 2 == 0)
                  {
                     for (int j = 0; j < treeToExpand.getPathNodes().size(); j++)
                        optimalPath.add(treeToExpand.getPathNodes().get(j));
                     for (int j = treeNotToExpand.getPathNodes().size() - 1; j > -1; j--)
                        optimalPath.add(treeNotToExpand.getPathNodes().get(j));
                  }
                  else
                  {
                     for (int j = 0; j < treeNotToExpand.getPathNodes().size(); j++)
                        optimalPath.add(treeNotToExpand.getPathNodes().get(j));
                     for (int j = treeToExpand.getPathNodes().size() - 1; j > -1; j--)
                        optimalPath.add(treeToExpand.getPathNodes().get(j));
                  }
                  pathNodes = new ArrayList<RRTNode>(optimalPath);

                  return true;
               }
            }
         }

      }
      return false;
   }

   public RRTNode getClosestNode(RRTNode node, RRTTree tree)
   {
      double distance = Double.MAX_VALUE;
      int indexOfClosestNode = 0;

      for (int i = 0; i < tree.getWholeNodes().size(); i++)
      {
         if (distance > node.getDistance(tree.getWholeNodes().get(i)))
         {
            distance = node.getDistance(tree.getWholeNodes().get(i));
            indexOfClosestNode = i;
         }
      }

      return tree.getWholeNodes().get(indexOfClosestNode);
   }

   public void updateOptimalPath(int sizeOfPiecewisePath)
   {
      if (optimalPath.size() > 1)
      {
         rrtPiecewisePath = new RRTPiecewisePath(optimalPath, sizeOfPiecewisePath);

         if (rrtPiecewisePath.updateShortCutPath() == true)
         {
            optimalPath = rrtPiecewisePath.getShortCutPath();
         }
      }
      else
      {
      }
   }

   public void updateOptimalPath(int sizeOfPiecewisePath, int numberOfIteration)
   {
      PrintTools.info("Buliding Started");
      for (int i = 0; i < numberOfIteration; i++)
      {
         updateOptimalPath(sizeOfPiecewisePath);
      }

      if (optimalPath.size() > 2)
      {
         ArrayList<RRTNode> tempPath = new ArrayList<RRTNode>();
         tempPath.add(optimalPath.get(0));
         for (int i = 1; i < optimalPath.size(); i++)
         {
            if (optimalPath.get(0).getDistance(optimalPath.get(i)) > 0)
            {
               tempPath.add(optimalPath.get(i));
            }
         }
         optimalPath = tempPath;
      }

      PrintTools.info("OptimalPath is Built");
   }
}
