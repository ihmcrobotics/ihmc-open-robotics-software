package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepGraphNode
{
   private final DiscreteFootstep endStep;
   private final DiscreteFootstep startStep;
   private Pose2D midFootPose = null;

   private final int hashCode;

   public FootstepGraphNode(DiscreteFootstep endStep, DiscreteFootstep startStep)
   {
      checkDifferentSides(endStep, startStep);
      this.endStep = endStep;
      this.startStep = startStep;
      this.hashCode = computeHashCode(this);
   }

   public DiscreteFootstep getEndStep()
   {
      return endStep;
   }

   public DiscreteFootstep getStartStep()
   {
      return startStep;
   }

   public RobotSide getEndSide()
   {
      return endStep.getRobotSide();
   }

   public RobotSide getStartSide()
   {
      return startStep.getRobotSide();
   }

   public Pose2D getOrComputeMidFootPose()
   {
      if (midFootPose == null)
      {
         midFootPose = new Pose2D();
         midFootPose.setX(EuclidCoreTools.interpolate(endStep.getX(), startStep.getX(), 0.5));
         midFootPose.setY(EuclidCoreTools.interpolate(endStep.getY(), startStep.getY(), 0.5));
         midFootPose.setYaw(AngleTools.interpolateAngle(endStep.getYaw(), startStep.getYaw(), 0.5));
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
      if(!startStep.equals(other.startStep))
         return false;
      return true;
   }

   private static int computeHashCode(FootstepGraphNode node)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + node.endStep.hashCode();
      result = prime * result + node.startStep.hashCode();
      return result;
   }

   @Override
   public String toString()
   {
      return "Footstep graph node:\n\t End " + endStep.toString() + "\n\t Start" + startStep.toString();
   }

   private static void checkDifferentSides(DiscreteFootstep endStep, DiscreteFootstep startStep)
   {
      if (endStep.getRobotSide() == startStep.getRobotSide())
         throw new RuntimeException("Footstep graph node given two steps on the same side!");
   }
}
