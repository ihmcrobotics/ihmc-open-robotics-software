package us.ihmc.quadrupedFootstepPlanning.ui;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class SimpleFootstepNode
{
   private final RobotQuadrant movingQuadrant;
   private final int xIndex;
   private final int yIndex;

   private final int hashCode;

   public SimpleFootstepNode(PawNode node)
   {
      this(node.getMovingQuadrant(), node.getXIndex(node.getMovingQuadrant()), node.getYIndex(node.getMovingQuadrant()));
   }

   public SimpleFootstepNode(RobotQuadrant movingQuadrant, double xPosition, double yPosition)
   {
      this(movingQuadrant, PawNode.snapToGrid(xPosition), PawNode.snapToGrid(yPosition));
   }

   public SimpleFootstepNode(RobotQuadrant movingQuadrant, int xIndex, int yIndex)
   {
      this.movingQuadrant = movingQuadrant;
      this.xIndex = xIndex;
      this.yIndex = yIndex;

      hashCode = computeHashCode(this);
   }

   private static int computeHashCode(SimpleFootstepNode node)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((node.movingQuadrant == null) ? 0 : node.movingQuadrant.hashCode());
      result = prime * result + node.xIndex;
      result = prime * result + node.yIndex;
      return result;
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   public RobotQuadrant getMovingQuadrant()
   {
      return movingQuadrant;
   }

   public int getXIndex()
   {
      return xIndex;
   }

   public int getYIndex()
   {
      return yIndex;
   }
}
