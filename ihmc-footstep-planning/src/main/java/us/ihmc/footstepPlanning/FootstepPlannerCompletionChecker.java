package us.ihmc.footstepPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerHeuristicCalculator;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.AStarFootstepPlannerIterationConductor;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.*;
import java.util.function.Predicate;

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
   private final List<FootstepNode> childNodes = new ArrayList<>();

   private double goalDistanceProximity;
   private double goalYawProximity;

   private FootstepNode startNode, endNode;
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

   public void initialize(FootstepNode startNode, SideDependentList<FootstepNode> goalNodes, double goalDistanceProximity, double goalYawProximity)
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

   public boolean checkIfGoalIsReached(AStarIterationData<FootstepNode> iterationData)
   {
      boolean proximityMode = goalDistanceProximity > 0.0 || goalYawProximity > 0.0;

      if (proximityMode)
      {
         iterationData.getValidChildNodes().sort(goalProximityComparator);
         for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
         {
            FootstepNode childNode = iterationData.getValidChildNodes().get(i);
            midFootPose.set(childNode.getOrComputeMidFootPoint(footstepPlannerParameters.getIdealFootstepWidth()), childNode.getYaw());

            boolean validYawProximity = midFootPose.getOrientation().distance(goalMidFootPose.getOrientation()) <= goalYawProximity;
            boolean validDistanceProximity = midFootPose.getPosition().distanceSquared(goalMidFootPose.getPosition()) <= MathTools.square(goalDistanceProximity);

            if (validYawProximity && validDistanceProximity && searchForSquaredUpStepInProximity(iterationData.getParentNode(), childNode))
            {
               return true;
            }
         }
      }
      else
      {
         for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
         {
            FootstepNode childNode = iterationData.getValidChildNodes().get(i);
            FootstepNode goalNode = goalNodes.get(childNode.getRobotSide());
            if (childNode.equals(goalNode))
            {
               endNode = goalNodes.get(childNode.getRobotSide().getOppositeSide());
               iterationConductor.getGraph().checkAndSetEdge(childNode, endNode, 0.0);
               return true;
            }
         }
      }

      for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
      {
         FootstepNode childNode = iterationData.getValidChildNodes().get(i);

         double cost = iterationConductor.getGraph().getCostFromStart(childNode) + heuristics.compute(childNode);
         if (cost < endNodeCost || endNode.equals(startNode))
         {
            endNode = childNode;
            endNodeCost = cost;
            endNodePathSize = iterationConductor.getGraph().getPathLengthFromStart(endNode);
         }
      }

      return false;
   }

   /**
    * Checks if this step can be followed by a square-up step that's also within the proximity bounds.
    * If so, the square-up step is set as the end nodeInProximity and the edge cost is set to zero.
    *
    * @param nodeInProximity step that's within goal proximity
    * @return if a square-up step was found
    */
   private boolean searchForSquaredUpStepInProximity(FootstepNode parentNode, FootstepNode nodeInProximity)
   {
      DirectedGraph<FootstepNode> graph = iterationConductor.getGraph();
      if (!graph.getOutgoingEdges().containsKey(nodeInProximity))
      {
         // this step hasn't been expanded yet, perform iteration now
         iterationConductor.doPlanningIteration(nodeInProximity, false);
      }

      // grab all outgoing edges
      childNodes.clear();
      Predicate<GraphEdge<FootstepNode>> validEdgeFilter = edge -> Double.isFinite(graph.getEdgeCostMap().get(edge).getEdgeCost());
      graph.getOutgoingEdges().get(nodeInProximity).stream().filter(validEdgeFilter).forEach(edge -> childNodes.add(edge.getEndNode()));

      if (childNodes.isEmpty())
         return false;

      squaredUpStepComparator.squaredUpStep.set(nodeInProximity.getX(), nodeInProximity.getY(), nodeInProximity.getYaw());
      squaredUpStepComparator.squaredUpStep.appendTranslation(0.0, nodeInProximity.getRobotSide().negateIfLeftSide(footstepPlannerParameters.getIdealFootstepWidth()));
      childNodes.sort(squaredUpStepComparator);

      for (int i = 0; i < childNodes.size(); i++)
      {
         FootstepNode closestStepToSquaredUp = childNodes.get(i);
         double distanceFromSquaredUpSquared = EuclidCoreTools.normSquared(closestStepToSquaredUp.getX() - squaredUpStepComparator.squaredUpStep.getX(),
                                                                           closestStepToSquaredUp.getY() - squaredUpStepComparator.squaredUpStep.getY());
         double angleFromSquaredUp = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(closestStepToSquaredUp.getYaw(),
                                                                                         squaredUpStepComparator.squaredUpStep.getYaw()));

         if (distanceFromSquaredUpSquared < MathTools.square(distanceEpsilonForSquaredUp) && angleFromSquaredUp < yawEpsilonForSquaredUp)
         {
            graph.updateEdgeCost(parentNode, nodeInProximity, 0.0);
            graph.updateEdgeCost(nodeInProximity, closestStepToSquaredUp, 0.0);
            endNode = closestStepToSquaredUp;
            return true;
         }
      }

      return false;
   }

   public FootstepNode getEndNode()
   {
      return endNode;
   }

   public int getEndNodePathSize()
   {
      return endNodePathSize;
   }

   private class GoalProximityComparator implements Comparator<FootstepNode>
   {
      private final FootstepPlannerParametersBasics footstepPlannerParameters;

      public GoalProximityComparator(FootstepPlannerParametersBasics footstepPlannerParameters)
      {
         this.footstepPlannerParameters = footstepPlannerParameters;
      }

      public int compare(FootstepNode nodeA, FootstepNode nodeB)
      {
         Point2D midFootPointA = nodeA.getOrComputeMidFootPoint(footstepPlannerParameters.getIdealFootstepWidth());
         Point2D midFootPointB = nodeB.getOrComputeMidFootPoint(footstepPlannerParameters.getIdealFootstepWidth());

         double positionDistanceA = midFootPointA.distanceSquared(goalMidFootPose.getPosition());
         double positionDistanceB = midFootPointB.distanceSquared(goalMidFootPose.getPosition());

         double yawScalar = 3.0;
         double yawDistanceA = yawScalar * Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(goalMidFootPose.getYaw(), nodeA.getYaw()));
         double yawDistanceB = yawScalar * Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(goalMidFootPose.getYaw(), nodeA.getYaw()));

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
