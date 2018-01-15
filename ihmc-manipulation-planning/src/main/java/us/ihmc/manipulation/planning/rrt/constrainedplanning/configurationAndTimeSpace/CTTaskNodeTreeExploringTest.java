package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.commons.PrintTools;

public class CTTaskNodeTreeExploringTest
{
   int numberOfTasks = 5;

   CTTaskNode rootNode;

   CTTaskNodeTree tree;

   TaskRegion taskRegion;

   CTTreeVisualizer treeVisualizer;

   public CTTaskNodeTreeExploringTest()
   {
      rootNode = new CTTaskNode(numberOfTasks + 1);
      rootNode.setNodeData(0, 0.0);

      for (int i = 1; i < numberOfTasks + 1; i++)
         rootNode.setNodeData(i, 0.5);
      
      
      rootNode.setNodeData(1, 0.1);
      rootNode.setNodeData(3, 0.9);
      rootNode.setNodeData(4, 0.9);

      taskRegion = new TaskRegion(numberOfTasks + 1);

      for (int i = 0; i < numberOfTasks + 1; i++)
         taskRegion.setRandomRegion(i, 0.0, 1.0);

      rootNode.convertDataToNormalizedData(taskRegion);

      tree = new CTTaskNodeTree(rootNode);

      tree.setTaskRegion(taskRegion);

      treeVisualizer = new CTTreeVisualizer(tree, true);
      treeVisualizer.initialize();

      treeVisualizer.update(rootNode);

      for (int i = 0; i < 1000; i++)
      {
         expandTree();
         if (tree.getNewNode().getTime() == taskRegion.getTrajectoryTime())
         {
            PrintTools.info("Finished " + i);
            break;
         }  
      }
   }

   private void expandTree()
   {
      tree.updateRandomConfiguration();

      tree.updateNearestNodeTaskTime();
      tree.updateNewConfiguration();
      tree.getNewNode().convertNormalizedDataToData(taskRegion);
      tree.getNewNode().setParentNode(tree.getNearNode());

      updateValidity(tree.getNewNode());
      if (tree.getNewNode().getValidity())
      {
         tree.connectNewNode(true);
         if (tree.getNewNode().getTime() == taskRegion.getTrajectoryTime())
         {
         }
      }
      else
      {         
         tree.connectNewNode(false);
                  
         tree.updateNearestNodeTaskOnly();         
         
         tree.updateNewConfiguration();
         tree.getNewNode().convertNormalizedDataToData(taskRegion);
         
         tree.getNewNode().setParentNode(tree.getNearNode());

         updateValidity(tree.getNewNode());
         if (tree.getNewNode().getValidity())
         {            
            tree.connectNewNode(true);
            if (tree.getNewNode().getTime() == taskRegion.getTrajectoryTime())
            {
            }
         }
      }

      treeVisualizer.update(tree.getNewNode());
   }

   private void updateValidity(CTTaskNode node)
   {
      if (0.4 < node.getNormalizedNodeData(0) && node.getNormalizedNodeData(0) < 0.6)
      {
         boolean eq1 = 0.3 < node.getNormalizedNodeData(1) && node.getNormalizedNodeData(1) < 0.7;
         boolean eq2 = 0.4 < node.getNormalizedNodeData(2) && node.getNormalizedNodeData(2) < 0.6;
         if (eq1 || eq2)
         {
            node.setValidity(false);
         }
      }

      if (0.8 < node.getNormalizedNodeData(0) && node.getNormalizedNodeData(0) < 0.95)
      {
         boolean eq1 = 0.1 < node.getNormalizedNodeData(1) && node.getNormalizedNodeData(1) < 0.4;
         if (eq1)
         {
            node.setValidity(false);
         }
      }
      
   }

   public static void main(String[] args)
   {
      System.out.println("Exploring test for CTNodeTree");

      CTTaskNodeTreeExploringTest test = new CTTaskNodeTreeExploringTest();

      System.out.println("Finished");
   }
}
