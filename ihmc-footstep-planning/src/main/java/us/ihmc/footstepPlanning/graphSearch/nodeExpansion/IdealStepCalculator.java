package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;

public class IdealStepCalculator
{
   private static final double distanceToleranceToMatchHeading = 0.25;
   private static final double yawToleranceToTurnTowardsGoal = 0.35;
   private static final double idealStepLengthWhenUpOrDown = 0.25;
   private static final double upOrDownStepThreshold = 0.2;

   private final HashMap<FootstepNode, FootstepNode> idealStepMap = new HashMap<>();
   private final FootstepPlannerParametersReadOnly parameters;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder;
   private SideDependentList<FootstepNode> goalNodes;
   private final FootstepNodeChecker nodeChecker;

   private final Pose2D goalMidFootPose = new Pose2D();
   private final Pose2D idealStep = new Pose2D();
   private final Pose3D projectionPose = new Pose3D();
   private double pathLength;

   public IdealStepCalculator(FootstepPlannerParametersReadOnly parameters, FootstepNodeChecker nodeChecker, WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder)
   {
      this.parameters = parameters;
      this.nodeChecker = nodeChecker;
      this.bodyPathPlanHolder = bodyPathPlanHolder;
   }

   public void initialize(SideDependentList<FootstepNode> goalNodes)
   {
      this.goalNodes = goalNodes;
      idealStepMap.clear();
      pathLength = bodyPathPlanHolder.computePathLength(0.0);

      Pose2D leftGoalPose = new Pose2D(goalNodes.get(RobotSide.LEFT).getX(), goalNodes.get(RobotSide.LEFT).getY(), goalNodes.get(RobotSide.LEFT).getYaw());
      Pose2D rightGoalPose = new Pose2D(goalNodes.get(RobotSide.RIGHT).getX(), goalNodes.get(RobotSide.RIGHT).getY(), goalNodes.get(RobotSide.RIGHT).getYaw());
      goalMidFootPose.interpolate(leftGoalPose, rightGoalPose, 0.5);
   }

   public FootstepNode computeIdealStep(FootstepNode stanceNode)
   {
      return idealStepMap.computeIfAbsent(stanceNode, this::computeIdealStepInternal);
   }

   private FootstepNode computeIdealStepInternal(FootstepNode stanceNode)
   {
      FootstepNode goalNode = goalNodes.get(stanceNode.getRobotSide().getOppositeSide());
      if (nodeChecker.isNodeValid(goalNode, stanceNode))
      {
         return goalNode;
      }

      Point2D midFootPoint = stanceNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      RobotSide stanceSide = stanceNode.getRobotSide();
      double alphaMidFoot = bodyPathPlanHolder.getClosestPoint(midFootPoint, projectionPose);
      int segmentIndex = bodyPathPlanHolder.getSegmentIndexFromAlpha(alphaMidFoot);
      bodyPathPlanHolder.getPointAlongPath(alphaMidFoot, projectionPose);

      double distanceFromGoalSquared = midFootPoint.distanceSquared(goalMidFootPose.getPosition());
      double distanceFromPathSquared = midFootPoint.distanceXYSquared(projectionPose.getPosition());
      double finalTurnProximity = parameters.getFinalTurnProximity();

      double desiredYaw;
      if (distanceFromGoalSquared < MathTools.square(finalTurnProximity))
      {
         desiredYaw = goalMidFootPose.getYaw();
      }
      else if (distanceFromPathSquared < MathTools.square(distanceToleranceToMatchHeading))
      {
         desiredYaw = bodyPathPlanHolder.getSegmentYaw(segmentIndex);
      }
      else
      {
         int numberOfCorrectiveStepsWhenOffPath = 2;
         double alphaLookAhead = MathTools.clamp(alphaMidFoot + numberOfCorrectiveStepsWhenOffPath * parameters.getIdealFootstepLength() / pathLength, 0.0, 1.0);
         bodyPathPlanHolder.getPointAlongPath(alphaLookAhead, projectionPose);
         desiredYaw = Math.atan2(projectionPose.getY() - midFootPoint.getY(), projectionPose.getX() - midFootPoint.getX());
      }

      double deltaYaw = AngleTools.computeAngleDifferenceMinusPiToPi(desiredYaw, stanceNode.getYaw());
      RobotSide stepSide = stanceSide.getOppositeSide();
      double yawLowerLimit = stepSide == RobotSide.LEFT ? parameters.getMinimumStepYaw() : - parameters.getMaximumStepYaw();
      double yawUpperLimit = stepSide == RobotSide.LEFT ? parameters.getMaximumStepYaw() : - parameters.getMinimumStepYaw();
      double achievableStepYaw = MathTools.clamp(deltaYaw, yawLowerLimit, yawUpperLimit);

      if (distanceFromGoalSquared <= MathTools.square(finalTurnProximity))
      {
         // turn in place at goal
         return turnInPlaceStep(stanceNode, goalMidFootPose.getPosition(), stanceSide, 0.5 * parameters.getIdealFootstepWidth(), achievableStepYaw);
      }
      else if (Math.abs(deltaYaw) > yawToleranceToTurnTowardsGoal)
      {
         // turn in place towards goal
         return turnInPlaceStep(stanceNode, midFootPoint, stanceSide, 0.5 * parameters.getIdealFootstepWidth(), achievableStepYaw);
      }
      else
      {
         // check if up or down step
         double alphaLookBack = MathTools.clamp(alphaMidFoot - parameters.getIdealFootstepLength() / pathLength, 0.0, 1.0);
         bodyPathPlanHolder.getPointAlongPath(alphaLookBack, projectionPose);
         double previousStepHeight = projectionPose.getZ();
         double alphaLookAhead = MathTools.clamp(alphaMidFoot +  parameters.getIdealFootstepLength() / pathLength, 0.0, 1.0);
         bodyPathPlanHolder.getPointAlongPath(alphaLookAhead, projectionPose);
         double nextStepHeight = projectionPose.getZ();
         double idealStepLength = Math.abs(nextStepHeight - previousStepHeight) > upOrDownStepThreshold ? idealStepLengthWhenUpOrDown : parameters.getIdealFootstepLength();
         
         // do ideal step with turn
         idealStep.set(midFootPoint, stanceNode.getYaw());
         idealStep.appendTranslation(idealStepLength, 0.0);
         idealStep.appendRotation(achievableStepYaw);

         // calculate step width
         boolean onLeftSideOfLine = bodyPathPlanHolder.getSegmentLine(segmentIndex).isPointOnLeftSideOfLine(idealStep.getPosition());
         double distanceFromPath = bodyPathPlanHolder.getSegmentLine(segmentIndex).distance(idealStep.getPosition());
         double maximumCorrectiveWidth = 0.05;
         double correctiveWidth = Math.min(maximumCorrectiveWidth, distanceFromPath) * (onLeftSideOfLine ? -1.0 : 1.0);
         double stepWidth = stanceSide.negateIfLeftSide(parameters.getIdealFootstepWidth());
         idealStep.appendTranslation(0.0, stepWidth + correctiveWidth);

         return new FootstepNode(idealStep.getX(), idealStep.getY(), idealStep.getYaw(), stanceNode.getRobotSide().getOppositeSide());
      }
   }

   private static FootstepNode turnInPlaceStep(FootstepNode startNode, Point2DBasics midFootPoint, RobotSide stanceSide, double idealFootstepWidth, double turnYaw)
   {
      Pose2D idealStep = new Pose2D(midFootPoint, startNode.getYaw());
      idealStep.appendRotation(turnYaw);
      idealStep.appendTranslation(0.0, stanceSide.negateIfLeftSide(idealFootstepWidth));
      return new FootstepNode(idealStep.getX(), idealStep.getY(), idealStep.getYaw(), startNode.getRobotSide().getOppositeSide());
   }
}