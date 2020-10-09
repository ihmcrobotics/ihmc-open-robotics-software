package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
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
   private static final double distanceEpsilonForSquaredUp = 0.06;
   private static final double yawEpsilonForSquaredUp = Math.toRadians(20.0);

   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final AStarFootstepPlannerIterationConductor iterationConductor;
   private final FootstepPlannerHeuristicCalculator heuristics;

   private final SquaredUpStepComparator squaredUpStepComparator;
   private final GoalProximityComparator goalProximityComparator;

   private final Pose2D goalMidFootPose = new Pose2D();
   private final Pose2D midFootPose = new Pose2D();
   private final List<FootstepGraphNode> childNodes = new ArrayList<>();

   private double goalDistanceProximity;
   private double goalYawProximity;

   private FootstepGraphNode startNode, endNode;
   private int endNodePathSize;
   private SideDependentList<FootstepNode> goalNodes;
   private double endNodeCost;

   public FootstepPlannerCompletionChecker(FootstepPlannerParametersBasics footstepPlannerParameters,
                                           AStarFootstepPlannerIterationConductor iterationConductor,
                                           FootstepPlannerHeuristicCalculator heuristics)
   {
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.iterationConductor = iterationConductor;
      this.heuristics = heuristics;

      squaredUpStepComparator = new SquaredUpStepComparator();
      goalProximityComparator = new GoalProximityComparator(footstepPlannerParameters);
   }

   public void initialize(FootstepGraphNode startNode, SideDependentList<FootstepNode> goalNodes, double goalDistanceProximity, double goalYawProximity)
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
      //      if (isProximityModeEnabled())
      //      {
      // TODO implement proximity mode with FootstepStanceNode
      //         iterationData.getValidChildNodes().sort(goalProximityComparator);
      //         for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
      //         {
      //            FootstepNode childNode = iterationData.getValidChildNodes().get(i);
      //            midFootPose.set(childNode.getOrComputeMidFootPoint(footstepPlannerParameters.getIdealFootstepWidth()), childNode.getYaw());
      //
      //            boolean validYawProximity = midFootPose.getOrientation().distance(goalMidFootPose.getOrientation()) <= goalYawProximity;
      //            boolean validDistanceProximity = midFootPose.getPosition().distanceSquared(goalMidFootPose.getPosition()) <= MathTools.square(goalDistanceProximity);
      //
      //            if (validYawProximity && validDistanceProximity && searchForSquaredUpStepInProximity(iterationData.getParentNode(), childNode))
      //            {
      //               return childNode;
      //            }
      //         }
      //      }
      //      else
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
      //      }

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

//   /**
//    * Checks if this step can be followed by a square-up step that's also within the proximity bounds.
//    * If so, the square-up step is set as the end nodeInProximity and the edge cost is set to zero.
//    *
//    * @param nodeInProximity step that's within goal proximity
//    * @return if a square-up step was found
//    */
//   private boolean searchForSquaredUpStepInProximity(FootstanceNode parentNode, FootstanceNode nodeInProximity)
//   {
//      DirectedGraph<FootstanceNode> graph = iterationConductor.getGraph();
//      if (!graph.getOutgoingEdges().containsKey(nodeInProximity))
//      {
//         // this step hasn't been expanded yet, perform iteration now
//         iterationConductor.doPlanningIteration(nodeInProximity, false);
//      }
//
//      // grab all outgoing edges
//      childNodes.clear();
//      Predicate<GraphEdge<FootstanceNode>> validEdgeFilter = edge -> Double.isFinite(graph.getEdgeCostMap().get(edge).getEdgeCost());
//      graph.getOutgoingEdges().get(nodeInProximity).stream().filter(validEdgeFilter).forEach(edge -> childNodes.add(edge.getEndNode()));
//
//      if (childNodes.isEmpty())
//         return false;
//
//      squaredUpStepComparator.squaredUpStep.set(nodeInProximity.getX(), nodeInProximity.getY(), nodeInProximity.getYaw());
//      squaredUpStepComparator.squaredUpStep.appendTranslation(0.0, nodeInProximity.getRobotSide().negateIfLeftSide(footstepPlannerParameters.getIdealFootstepWidth()));
//      childNodes.sort(squaredUpStepComparator);
//
//      for (int i = 0; i < childNodes.size(); i++)
//      {
//         FootstepNode closestStepToSquaredUp = childNodes.get(i);
//         double distanceFromSquaredUpSquared = EuclidCoreTools.normSquared(closestStepToSquaredUp.getX() - squaredUpStepComparator.squaredUpStep.getX(),
//                                                                           closestStepToSquaredUp.getY() - squaredUpStepComparator.squaredUpStep.getY());
//         double angleFromSquaredUp = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(closestStepToSquaredUp.getYaw(),
//                                                                                         squaredUpStepComparator.squaredUpStep.getYaw()));
//
//         if (distanceFromSquaredUpSquared < MathTools.square(distanceEpsilonForSquaredUp) && angleFromSquaredUp < yawEpsilonForSquaredUp)
//         {
//            graph.updateEdgeCost(parentNode, nodeInProximity, 0.0);
//            graph.updateEdgeCost(nodeInProximity, closestStepToSquaredUp, 0.0);
//            endNode = closestStepToSquaredUp;
//            return true;
//         }
//      }
//
//      return false;
//   }

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

   private class GoalProximityComparator implements Comparator<FootstepGraphNode>
   {
      private final FootstepPlannerParametersBasics footstepPlannerParameters;

      public GoalProximityComparator(FootstepPlannerParametersBasics footstepPlannerParameters)
      {
         this.footstepPlannerParameters = footstepPlannerParameters;
      }

      public int compare(FootstepGraphNode nodeA, FootstepGraphNode nodeB)
      {
         Pose2D midFootPoseA = nodeA.getOrComputeMidFootPose();
         Pose2D midFootPoseB = nodeB.getOrComputeMidFootPose();

         double positionDistanceA = midFootPoseA.getPosition().distanceSquared(goalMidFootPose.getPosition());
         double positionDistanceB = midFootPoseB.getPosition().distanceSquared(goalMidFootPose.getPosition());

         double yawScalar = 3.0;
         double yawDistanceA = yawScalar * Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(goalMidFootPose.getYaw(), nodeA.getEndStep().getYaw()));
         double yawDistanceB = yawScalar * Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(goalMidFootPose.getYaw(), nodeA.getEndStep().getYaw()));

         return Double.compare(positionDistanceA + yawDistanceA, positionDistanceB + yawDistanceB);
      }
   }

   private class SquaredUpStepComparator implements Comparator<FootstepNode>
   {
      private final Pose2D squaredUpStep = new Pose2D();

      public int compare(FootstepNode nodeA, FootstepNode nodeB)
      {
         double positionDistanceA = EuclidCoreTools.norm(nodeA.getX() - squaredUpStep.getX(), nodeA.getY() - squaredUpStep.getY());
         double positionDistanceB = EuclidCoreTools.norm(nodeB.getX() - squaredUpStep.getX(), nodeB.getY() - squaredUpStep.getY());

         double yawScalar = 3.0;
         double yawDistanceA = yawScalar * Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(nodeA.getYaw(), squaredUpStep.getYaw()));
         double yawDistanceB = yawScalar * Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(nodeB.getYaw(), squaredUpStep.getYaw()));

         return Double.compare(positionDistanceA + yawDistanceA, positionDistanceB + yawDistanceB);
      }
   }
}
