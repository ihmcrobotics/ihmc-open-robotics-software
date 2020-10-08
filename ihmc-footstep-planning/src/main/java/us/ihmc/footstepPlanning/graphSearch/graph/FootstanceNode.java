package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.robotics.robotSide.RobotSide;

public class FootstanceNode
{
   private final FootstepNode stanceNode;
   private final FootstepNode swingNode;

   private final int hashCode;

   public FootstanceNode(FootstepNode stanceNode, FootstepNode swingNode)
   {
      checkDifferentSides(stanceNode, swingNode);
      this.stanceNode = stanceNode;
      this.swingNode = swingNode;
      this.hashCode = computeHashCode(this);
   }

   public double getX()
   {
      return stanceNode.getX();
   }

   public double getY()
   {
      return stanceNode.getY();
   }

   public double getYaw()
   {
      return stanceNode.getYaw();
   }

   public int getXIndex()
   {
      return stanceNode.getXIndex();
   }

   public int getYIndex()
   {
      return stanceNode.getYIndex();
   }

   public int getYawIndex()
   {
      return stanceNode.getYawIndex();
   }

   public LatticeNode getStanceLatticeNode()
   {
      return stanceNode.getLatticeNode();
   }

   public LatticeNode getSwingLatticeNode()
   {
      return swingNode.getLatticeNode();
   }

   public FootstepNode getStanceNode()
   {
      return stanceNode;
   }

   public FootstepNode getSwingNode()
   {
      return swingNode;
   }

   public RobotSide getStanceSide()
   {
      return stanceNode.getRobotSide();
   }

   public RobotSide getSwingSide()
   {
      return swingNode.getRobotSide();
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      FootstanceNode other = (FootstanceNode) obj;
      if(!stanceNode.equals(other.stanceNode))
         return false;
      if(!swingNode.equals(other.swingNode))
         return false;
      return true;
   }

   private static int computeHashCode(FootstanceNode node)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + node.stanceNode.hashCode();
      result = prime * result + node.swingNode.hashCode();
      return result;
   }

   @Override
   public String toString()
   {
      return "Footstance node:\n\t Stance " + stanceNode.toString() + "\n\t Swing" + swingNode.toString();
   }

   private static void checkDifferentSides(FootstepNode stanceNode, FootstepNode swingNode)
   {
      if (stanceNode.getRobotSide() == swingNode.getRobotSide())
         throw new RuntimeException("Footstance node given two steps on the same side!");
   }
}
