package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepGraphNode
{
   private final FootstepNode endStep;
   private final FootstepNode startStart;
   private Pose2D midFootPose = null;

   private final int hashCode;

   public FootstepGraphNode(FootstepGraphNode parentNode, double childStepX, double childStepY, double childStepYaw)
   {
      this(new FootstepNode(childStepX, childStepY, childStepYaw, parentNode.getStartSide()), parentNode.getEndStep());
   }

   public FootstepGraphNode(FootstepGraphNode parentNode, int childStepXIndex, int childStepYIndex, int childStepYawIndex)
   {
      this(new FootstepNode(childStepXIndex, childStepYIndex, childStepYawIndex, parentNode.getStartSide()), parentNode.getEndStep());
   }

   public FootstepGraphNode(FootstepNode endStep, FootstepNode startStart)
   {
      checkDifferentSides(endStep, startStart);
      this.endStep = endStep;
      this.startStart = startStart;
      this.hashCode = computeHashCode(this);
   }

   public FootstepNode getEndStep()
   {
      return endStep;
   }

   public FootstepNode getStartStart()
   {
      return startStart;
   }

   public RobotSide getEndSide()
   {
      return endStep.getRobotSide();
   }

   public RobotSide getStartSide()
   {
      return startStart.getRobotSide();
   }

   public Pose2D getOrComputeMidFootPose()
   {
      if (midFootPose == null)
      {
         midFootPose = new Pose2D();
         midFootPose.setX(EuclidCoreTools.interpolate(endStep.getX(), startStart.getX(), 0.5));
         midFootPose.setY(EuclidCoreTools.interpolate(endStep.getY(), startStart.getY(), 0.5));
         midFootPose.setYaw(AngleTools.interpolateAngle(endStep.getYaw(), startStart.getYaw(), 0.5));
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
      FootstepGraphNode other = (FootstepGraphNode) obj;
      if(!endStep.equals(other.endStep))
         return false;
      if(!startStart.equals(other.startStart))
         return false;
      return true;
   }

   private static int computeHashCode(FootstepGraphNode node)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + node.endStep.hashCode();
      result = prime * result + node.startStart.hashCode();
      return result;
   }

   @Override
   public String toString()
   {
      return "Footstep graph node:\n\t End " + endStep.toString() + "\n\t Start" + startStart.toString();
   }

   private static void checkDifferentSides(FootstepNode endStep, FootstepNode startStep)
   {
      if (endStep.getRobotSide() == startStep.getRobotSide())
         throw new RuntimeException("Footstep graph node given two steps on the same side!");
   }
}
