package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerHeuristicCalculator;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.AStarFootstepPlannerIterationConductor;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.*;

public class FootstepPlannerCompletionChecker
{
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final AStarFootstepPlannerIterationConductor iterationConductor;
   private final FootstepPlannerHeuristicCalculator heuristics;

   private final Pose2D goalMidFootPose = new Pose2D();
   private double goalDistanceProximity;
   private double goalYawProximity;

   private FootstepGraphNode startNode, endNode;
   private int endNodePathSize;
   private SideDependentList<DiscreteFootstep> goalNodes;
   private double endNodeCost;

   private final SquaredUpStepComparator squaredUpStepComparator = new SquaredUpStepComparator();

   public FootstepPlannerCompletionChecker(FootstepPlannerParametersBasics footstepPlannerParameters,
                                           AStarFootstepPlannerIterationConductor iterationConductor,
                                           FootstepPlannerHeuristicCalculator heuristics)
   {
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.iterationConductor = iterationConductor;
      this.heuristics = heuristics;
   }

   public void initialize(FootstepGraphNode startNode, SideDependentList<DiscreteFootstep> goalNodes, double goalDistanceProximity, double goalYawProximity)
   {
      this.startNode = startNode;
      this.goalNodes = goalNodes;
      this.endNodePathSize = 0;

      this.goalDistanceProximity = goalDistanceProximity;
      this.goalYawProximity = goalYawProximity;

      endNode = startNode;
      endNodeCost = heuristics.compute(startNode);

      goalMidFootPose.setX(0.5 * (goalNodes.get(RobotSide.LEFT).getX() + goalNodes.get(RobotSide.RIGHT).getX()));
      goalMidFootPose.setY(0.5 * (goalNodes.get(RobotSide.LEFT).getY() + goalNodes.get(RobotSide.RIGHT).getY()));
      goalMidFootPose.setYaw(AngleTools.interpolateAngle(goalNodes.get(RobotSide.LEFT).getYaw(), goalNodes.get(RobotSide.RIGHT).getYaw(), 0.5));
   }

   /**
    * Checks if goal is reachable. If it is, the goal step is appended by a goal step on the opposite side and the expanded step is returned.
    * If the goal is not reached this returns null
    */
   public FootstepGraphNode checkIfGoalIsReached(AStarIterationData<FootstepGraphNode> iterationData)
   {
      if (isProximityModeEnabled() && iterationData.getParentNode() != null)
      {
         DiscreteFootstep stanceStep = iterationData.getParentNode().getEndStep();
         squaredUpStepComparator.setIdealSquaredUpStep(stanceStep, footstepPlannerParameters.getIdealFootstepWidth());
         iterationData.getValidChildNodes().sort(squaredUpStepComparator);
         FootstepGraphNode candidateStepInProximity = iterationData.getValidChildNodes().get(0);
         Pose2D midFootPose = candidateStepInProximity.getOrComputeMidFootPose();

         boolean validPosition = goalDistanceProximity < 0.0 || midFootPose.getPosition().distance(goalMidFootPose.getPosition()) < goalDistanceProximity;
         boolean validOrientation = goalYawProximity < 0.0 || midFootPose.getOrientation().distance(goalMidFootPose.getOrientation()) < goalYawProximity;

         if (validPosition && validOrientation)
         {
            endNode = candidateStepInProximity;
            return endNode;
         }
      }
      else
      {
         for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
         {
            FootstepGraphNode childNode = iterationData.getValidChildNodes().get(i);
            if (childNode.getEndStep().equals(goalNodes.get(childNode.getEndSide())))
            {
               endNode = new FootstepGraphNode(goalNodes.get(childNode.getStartSide()), goalNodes.get(childNode.getEndSide()));
               iterationConductor.getGraph().checkAndSetEdge(childNode, endNode, 0.0);
               return childNode;
            }
         }
      }

      for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
      {
         FootstepGraphNode childNode = iterationData.getValidChildNodes().get(i);

         double cost = heuristics.compute(childNode);
         if (cost < endNodeCost || endNode.equals(startNode))
         {
            endNode = childNode;
            endNodeCost = cost;
            endNodePathSize = iterationConductor.getGraph().getPathLengthFromStart(endNode);
         }
      }

      return null;
   }

   public boolean isProximityModeEnabled()
   {
      return goalDistanceProximity > 0.0 || goalYawProximity > 0.0;
   }

   public FootstepGraphNode getEndNode()
   {
      return endNode;
   }

   public int getEndNodePathSize()
   {
      return endNodePathSize;
   }

   private static class SquaredUpStepComparator implements Comparator<FootstepGraphNode>
   {
      private final Pose2D squaredUpStep = new Pose2D();

      public int compare(FootstepGraphNode nodeA, FootstepGraphNode nodeB)
      {
         double positionDistanceA = EuclidCoreTools.norm(nodeA.getEndStep().getX() - squaredUpStep.getX(), nodeA.getEndStep().getY() - squaredUpStep.getY());
         double positionDistanceB = EuclidCoreTools.norm(nodeB.getEndStep().getX() - squaredUpStep.getX(), nodeB.getEndStep().getY() - squaredUpStep.getY());

         double yawScalar = 3.0;
         double yawDistanceA = yawScalar * nodeA.getStanceAngle();
         double yawDistanceB = yawScalar * nodeB.getStanceAngle();

         return Double.compare(positionDistanceA + yawDistanceA, positionDistanceB + yawDistanceB);
      }

      void setIdealSquaredUpStep(DiscreteFootstep footstep, double idealStepWidth)
      {
         double stepX = footstep.getRobotSide().negateIfRightSide(Math.sin(footstep.getYaw()) * idealStepWidth);
         double stepY = -footstep.getRobotSide().negateIfRightSide(Math.cos(footstep.getYaw()) * idealStepWidth);
         squaredUpStep.setX(footstep.getX() + stepX);
         squaredUpStep.setY(footstep.getY() + stepY);
         squaredUpStep.setYaw(footstep.getYaw());
      }
   }
}
