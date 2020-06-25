package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.footstepPlanning.FootstepPlanHeading;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;
import java.util.function.BiPredicate;

public class IdealStepCalculator
{
   // TODO extract these to parameters once they're stable
   private static final double idealStepLengthWhenUpOrDownMultiplier = 0.7;
   private static final double maxDistanceAdjustmentTowardsPath = 0.15;
   private static final double maxYawAdjustmentTowardsPath = Math.toRadians(20.0);

   private final HashMap<FootstepNode, FootstepNode> idealStepMap = new HashMap<>();
   private final FootstepPlannerParametersReadOnly parameters;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder;
   private final BiPredicate<FootstepNode, FootstepNode> nodeChecker;

   private SideDependentList<FootstepNode> goalNodes;
   private PlanarRegionsList planarRegionsList;
   private FootstepPlanHeading desiredHeading = FootstepPlanHeading.FORWARD;

   private final Pose2D goalMidFootPose = new Pose2D();
   private final Pose2D idealStep = new Pose2D();
   private final Pose3D projectionPose = new Pose3D();
   private double pathLength;

   public IdealStepCalculator(FootstepPlannerParametersReadOnly parameters, BiPredicate<FootstepNode, FootstepNode> nodeChecker, WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder)
   {
      this.parameters = parameters;
      this.nodeChecker = nodeChecker;
      this.bodyPathPlanHolder = bodyPathPlanHolder;
   }

   public void initialize(SideDependentList<FootstepNode> goalNodes, FootstepPlanHeading desiredHeading)
   {
      this.goalNodes = goalNodes;
      this.desiredHeading = desiredHeading;

      idealStepMap.clear();
      pathLength = bodyPathPlanHolder.computePathLength(0.0);

      Pose2D leftGoalPose = new Pose2D(goalNodes.get(RobotSide.LEFT).getX(), goalNodes.get(RobotSide.LEFT).getY(), goalNodes.get(RobotSide.LEFT).getYaw());
      Pose2D rightGoalPose = new Pose2D(goalNodes.get(RobotSide.RIGHT).getX(), goalNodes.get(RobotSide.RIGHT).getY(), goalNodes.get(RobotSide.RIGHT).getYaw());
      goalMidFootPose.interpolate(leftGoalPose, rightGoalPose, 0.5);
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public FootstepNode computeIdealStep(FootstepNode stanceNode)
   {
      return idealStepMap.computeIfAbsent(stanceNode, this::computeIdealStepInternal);
   }

   private boolean flatGroundMode()
   {
      return planarRegionsList == null || planarRegionsList.isEmpty();
   }

   private FootstepNode computeIdealStepInternal(FootstepNode stanceNode)
   {
      boolean attemptSquareUp = attemptSquareUp(stanceNode);

      FootstepNode goalNode = goalNodes.get(stanceNode.getRobotSide().getOppositeSide());
      if (!flatGroundMode() && nodeChecker.test(goalNode, stanceNode) && !attemptSquareUp)
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

      if (attemptSquareUp)
      {
         double turnYaw = 0.0;
         return turnInPlaceStep(stanceNode, midFootPoint, stanceNode.getRobotSide(), 0.5 * parameters.getIdealFootstepWidth(), turnYaw);
      }

      double desiredYaw;
      if (distanceFromGoalSquared < MathTools.square(finalTurnProximity))
      {
         desiredYaw = goalMidFootPose.getYaw();
      }
      else if (distanceFromPathSquared < MathTools.square(parameters.getDistanceFromPathTolerance()))
      {
         desiredYaw = EuclidCoreTools.trimAngleMinusPiToPi(bodyPathPlanHolder.getSegmentYaw(segmentIndex) + desiredHeading.getYawOffset());
      }
      else
      {
         int numberOfCorrectiveStepsWhenOffPath = 2;
         double alphaLookAhead = MathTools.clamp(alphaMidFoot + numberOfCorrectiveStepsWhenOffPath * parameters.getIdealFootstepLength() / pathLength, 0.0, 1.0);
         bodyPathPlanHolder.getPointAlongPath(alphaLookAhead, projectionPose);
         desiredYaw = Math.atan2(projectionPose.getY() - midFootPoint.getY(), projectionPose.getX() - midFootPoint.getX());
         desiredYaw = EuclidCoreTools.trimAngleMinusPiToPi(desiredYaw + desiredHeading.getYawOffset());
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
      else if (Math.abs(deltaYaw) > parameters.getDeltaYawFromReferenceTolerance())
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
         double idealStepLengthMultiplier = Math.abs(nextStepHeight - previousStepHeight) > parameters.getMaximumStepZWhenSteppingUp() ? idealStepLengthWhenUpOrDownMultiplier : 1.0;

         if (desiredHeading == FootstepPlanHeading.FORWARD)
         {
            // do ideal step with turn
            double idealStepLength = idealStepLengthMultiplier * parameters.getIdealFootstepLength();
            idealStep.set(midFootPoint, stanceNode.getYaw());
            idealStep.appendTranslation(idealStepLength, 0.0);

            // calculate step width
            idealStep.appendRotation(achievableStepYaw);
            double correctiveWidth = calculateCorrectiveDistance(segmentIndex);
            double stepWidth = stanceSide.negateIfLeftSide(0.5 * parameters.getIdealFootstepWidth());
            idealStep.appendTranslation(0.0, stepWidth + correctiveWidth);

            // correct step yaw
            double yawAdjustment = calculateCorrectiveYaw(segmentIndex);
            idealStep.appendRotation(yawAdjustment);
         }
         else if (desiredHeading == FootstepPlanHeading.BACKWARD)
         {
            // do ideal step with turn
            double idealStepLength = idealStepLengthMultiplier * parameters.getIdealBackStepLength();
            idealStep.set(midFootPoint, stanceNode.getYaw());
            idealStep.appendTranslation(idealStepLength, 0.0);

            // calculate step width
            idealStep.appendRotation(achievableStepYaw);
            double correctiveWidth = calculateCorrectiveDistance(segmentIndex);
            double stepWidth = stanceSide.negateIfLeftSide(0.5 * parameters.getIdealFootstepWidth());
            idealStep.appendTranslation(0.0, stepWidth - correctiveWidth);
         }
         else if (desiredHeading == FootstepPlanHeading.LEFT)
         {
            // do ideal step with turn
            double idealStepWidth = stepSide == RobotSide.LEFT ? idealStepLengthMultiplier * parameters.getIdealSideStepWidth() : - parameters.getMinimumStepWidth();
            idealStep.set(stanceNode.getX(), stanceNode.getY(), stanceNode.getYaw());
            idealStep.appendTranslation(0.0, idealStepWidth);

            // calculate step length
            idealStep.appendRotation(achievableStepYaw);
            double correctiveLength = calculateCorrectiveDistance(segmentIndex);
            idealStep.appendTranslation(- correctiveLength, 0.0);
         }
         else if (desiredHeading == FootstepPlanHeading.RIGHT)
         {
            // do ideal step with turn
            double idealStepWidth = stepSide == RobotSide.RIGHT ? - idealStepLengthMultiplier * parameters.getIdealSideStepWidth() : parameters.getMinimumStepWidth();
            idealStep.set(stanceNode.getX(), stanceNode.getY(), stanceNode.getYaw());
            idealStep.appendTranslation(0.0, idealStepWidth);

            // calculate step length
            idealStep.appendRotation(achievableStepYaw);
            double correctiveLength = calculateCorrectiveDistance(segmentIndex);
            idealStep.appendTranslation(correctiveLength, 0.0);
         }

         return new FootstepNode(idealStep.getX(), idealStep.getY(), idealStep.getYaw(), stanceNode.getRobotSide().getOppositeSide());
      }
   }

   private boolean attemptSquareUp(FootstepNode stanceNode)
   {
      RobotSide requestedStepSide = parameters.getStepOnlyWithRequestedSide();
      return requestedStepSide != null && requestedStepSide != stanceNode.getRobotSide().getOppositeSide();
   }

   private static FootstepNode turnInPlaceStep(FootstepNode startNode, Point2DBasics midFootPoint, RobotSide stanceSide, double idealFootstepWidth, double turnYaw)
   {
      Pose2D idealStep = new Pose2D(midFootPoint, startNode.getYaw());
      idealStep.appendRotation(turnYaw);
      idealStep.appendTranslation(0.0, stanceSide.negateIfLeftSide(idealFootstepWidth));
      return new FootstepNode(idealStep.getX(), idealStep.getY(), idealStep.getYaw(), startNode.getRobotSide().getOppositeSide());
   }

   private double calculateCorrectiveDistance(int segmentIndex)
   {
      boolean onSideOfLine = bodyPathPlanHolder.getSegmentLine(segmentIndex).isPointOnLeftSideOfLine(idealStep.getPosition());
      double distanceFromPath = bodyPathPlanHolder.getSegmentLine(segmentIndex).distance(idealStep.getPosition());
      double clampedDistanceFromPath = Math.min(maxDistanceAdjustmentTowardsPath, distanceFromPath);

      double signedDistanceFromPath = clampedDistanceFromPath * (onSideOfLine ? 1.0 : -1.0);
      return - signedDistanceFromPath;
   }

   private double calculateCorrectiveYaw(int segmentIndex)
   {
      double currentYaw = idealStep.getYaw();
      double desiredYaw = bodyPathPlanHolder.getSegmentYaw(segmentIndex);
      double correctiveYaw = AngleTools.computeAngleDifferenceMinusPiToPi(desiredYaw, currentYaw);
      return MathTools.clamp(correctiveYaw, maxYawAdjustmentTowardsPath);
   }
}