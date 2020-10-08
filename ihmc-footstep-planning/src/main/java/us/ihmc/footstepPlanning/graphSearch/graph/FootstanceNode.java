package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstanceNode
{
   private final FootstepNode stanceNode;
   private final FootstepNode swingNode;
   private Pose2D midFootPose = null;

   private final int hashCode;

   public FootstanceNode(FootstanceNode parentNode, double stanceStepX, double stanceStepY, double stanceStepYaw)
   {
      this(new FootstepNode(stanceStepX, stanceStepY, stanceStepYaw, parentNode.getSwingSide()), parentNode.getStanceNode());
   }

   public FootstanceNode(FootstanceNode parentNode, int stanceStepXIndex, int stanceStepYIndex, int stanceStepYawIndex)
   {
      this(new FootstepNode(stanceStepXIndex, stanceStepYIndex, stanceStepYawIndex, parentNode.getSwingSide()), parentNode.getStanceNode());
   }

   public FootstanceNode(FootstepNode stanceNode, FootstepNode swingNode)
   {
      checkDifferentSides(stanceNode, swingNode);
      this.stanceNode = stanceNode;
      this.swingNode = swingNode;
      this.hashCode = computeHashCode(this);
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

   public Pose2D getOrComputeMidFootPose()
   {
      if (midFootPose == null)
      {
         midFootPose = new Pose2D();
         midFootPose.setX(EuclidCoreTools.interpolate(stanceNode.getX(), swingNode.getX(), 0.5));
         midFootPose.setY(EuclidCoreTools.interpolate(stanceNode.getY(), swingNode.getY(), 0.5));
         midFootPose.setYaw(AngleTools.interpolateAngle(stanceNode.getYaw(), swingNode.getYaw(), 0.5));
      }

      return midFootPose;
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
