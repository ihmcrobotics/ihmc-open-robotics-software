package us.ihmc.footstepPlanning.graphSearch.graph;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This object represents a node on the graph search by the footstep planner.
 * A node is a robot "stance", i.e. a left footstep and right footstep.
 * An edge represents a step, i.e. a transition between two stances.
 */
public class FootstepGraphNode
{
   /**
    * This is the start-of-swing step when stepping at this node. "First" refers to the step's sequencing
    * Note that this node's parent's secondStep will also be this step
    */
   private final DiscreteFootstep firstStep;
   /**
    * This is the stance step when stepping at this node. "Second" refers to the step's sequencing
    * Note that this node's children's firstStep will also be this step
    */
   private final DiscreteFootstep secondStep;

   private Pose2D midFootPose = null;
   private final int hashCode;

   public FootstepGraphNode(DiscreteFootstep firstStep, DiscreteFootstep secondStep)
   {
      checkDifferentSides(firstStep, secondStep);
      this.firstStep = firstStep;
      this.secondStep = secondStep;
      this.hashCode = computeHashCode(this);
   }

   public DiscreteFootstep getFirstStep()
   {
      return firstStep;
   }

   public DiscreteFootstep getSecondStep()
   {
      return secondStep;
   }

   public RobotSide getFirstStepSide()
   {
      return firstStep.getRobotSide();
   }

   public RobotSide getSecondStepSide()
   {
      return secondStep.getRobotSide();
   }

   public Pose2D getOrComputeMidFootPose()
   {
      if (midFootPose == null)
      {
         midFootPose = new Pose2D();
         midFootPose.setX(EuclidCoreTools.interpolate(secondStep.getX(), firstStep.getX(), 0.5));
         midFootPose.setY(EuclidCoreTools.interpolate(secondStep.getY(), firstStep.getY(), 0.5));
         midFootPose.setYaw(AngleTools.interpolateAngle(secondStep.getYaw(), firstStep.getYaw(), 0.5));
      }

      return midFootPose;
   }

   public double getStanceAngle()
   {
      return Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(firstStep.getYaw(), secondStep.getYaw()));
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
      if(!firstStep.equals(other.firstStep))
         return false;
      if(!secondStep.equals(other.secondStep))
         return false;
      return true;
   }

   private static int computeHashCode(FootstepGraphNode node)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + node.firstStep.hashCode();
      result = prime * result + node.secondStep.hashCode();
      return result;
   }

   @Override
   public String toString()
   {
      return "Footstep graph node:\n\t First " + firstStep.toString() + "\n\t Second" + secondStep.toString();
   }

   private static void checkDifferentSides(DiscreteFootstep firstStep, DiscreteFootstep secondStep)
   {
      if (firstStep.getRobotSide() == secondStep.getRobotSide())
         throw new RuntimeException("Footstep graph node given two steps on the same side!");
   }
}
