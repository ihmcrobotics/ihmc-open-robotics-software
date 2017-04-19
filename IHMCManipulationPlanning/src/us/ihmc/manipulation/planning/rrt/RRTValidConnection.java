package us.ihmc.manipulation.planning.rrt;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;

public class RRTValidConnection
{
   private RRTNode parentNode;
   private RRTNode childNode;
   private RRTNode nodeCreator;

   private ArrayList<RRTNode> nodes = new ArrayList<RRTNode>();

   public int numberOfPiece = 20;

   public RRTValidConnection(RRTNode parentNode, RRTNode childNode)
   {
      this.parentNode = parentNode;
      this.childNode = childNode;
      this.nodeCreator = parentNode.createNode();
      initialize();
   }

   private void initialize()
   {
      double length = parentNode.getDistance(childNode);

      if (length == 0)
      {
         nodes.add(parentNode);
      }
      else
      {
         for (int i = 1; i < numberOfPiece-1; i++)
         {
            RRTNode aNode = nodeCreator.createNode();

            for (int j = 0; j < parentNode.getDimensionOfNodeData(); j++)
            {
               double parentData = parentNode.getNodeData(j);
               double childData = childNode.getNodeData(j);
               double aNodeData;
               aNodeData = (parentData - childData) * i / (numberOfPiece - 1) + childData;
               aNode.setNodeData(j, aNodeData);
            }

            nodes.add(aNode);
         }
      }
   }

   public boolean isValidConnection()
   {
      for (int i = 1; i < nodes.size()-1; i++)
      {
         if (nodes.get(i).isValidNode() == false)
         {
            return false;
         }
      }
      return true;
   }
   /*
    * numberOfPiece should larger than 2
    */
   public void reInitialize(int numberOfPiece)
   {
      nodes.clear();
      this.numberOfPiece = numberOfPiece;
      initialize();
   }   
}
