package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.footstepPlanning.FootstepPlanHeading;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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
   private double desiredHeading = 0.0;

   private final SideDependentList<YoDouble> idealStepLengths = new SideDependentList<>();
   private final SideDependentList<YoDouble> idealStepWidths = new SideDependentList<>();

   private final YoDouble correctiveDistanceX;
   private final YoDouble correctiveDistanceY;
   private final YoDouble correctiveYaw;

   private final Pose2D goalMidFootPose = new Pose2D();
   private final Pose2D stanceFootPose = new Pose2D();
   private final Pose2D idealStep = new Pose2D();
   private final Pose3D projectionPose = new Pose3D();
   private final Pose2D idealMidFootPose = new Pose2D();
   private double pathLength;

   public IdealStepCalculator(FootstepPlannerParametersReadOnly parameters,
                              BiPredicate<FootstepNode, FootstepNode> nodeChecker,
                              WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder,
                              YoRegistry registry)
   {
      this.parameters = parameters;
      this.nodeChecker = nodeChecker;
      this.bodyPathPlanHolder = bodyPathPlanHolder;

      for (RobotSide robotSide : RobotSide.values)
      {
         idealStepLengths.put(robotSide, new YoDouble(robotSide.getShortLowerCaseName() + "_IdealStepLength", registry));
         idealStepWidths.put(robotSide, new YoDouble(robotSide.getShortLowerCaseName() + "-IdealStepWidth", registry));
      }

      correctiveDistanceX = new YoDouble("correctiveDistanceX", registry);
      correctiveDistanceY = new YoDouble("correctiveDistanceY", registry);
      correctiveYaw = new YoDouble("correctiveYaw", registry);
   }

   public void initialize(SideDependentList<FootstepNode> goalNodes, double desiredHeading)
   {
      this.goalNodes = goalNodes;
      this.desiredHeading = desiredHeading;

      idealStepMap.clear();
      pathLength = bodyPathPlanHolder.computePathLength(0.0);

      Pose2D leftGoalPose = new Pose2D(goalNodes.get(RobotSide.LEFT).getX(), goalNodes.get(RobotSide.LEFT).getY(), goalNodes.get(RobotSide.LEFT).getYaw());
      Pose2D rightGoalPose = new Pose2D(goalNodes.get(RobotSide.RIGHT).getX(), goalNodes.get(RobotSide.RIGHT).getY(), goalNodes.get(RobotSide.RIGHT).getYaw());
      goalMidFootPose.interpolate(leftGoalPose, rightGoalPose, 0.5);
      computeAdjustedIdealStepParameters();
   }

   private void computeAdjustedIdealStepParameters()
   {
      FootstepPlanHeading primaryDirection = FootstepPlanHeading.FORWARD;
      double closestAngle = Double.MAX_VALUE;
      for (FootstepPlanHeading heading : FootstepPlanHeading.values())
      {
         double deltaAngle = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(desiredHeading, heading.getYawOffset()));
         if (Math.abs(deltaAngle) < closestAngle)
         {
            closestAngle = deltaAngle;
            primaryDirection = heading;
         }
      }

      FootstepPlanHeading secondaryDirection = FootstepPlanHeading.FORWARD;
      double secondClosestAngle = Double.MAX_VALUE;
      for (FootstepPlanHeading heading : FootstepPlanHeading.values())
      {
         if (heading == primaryDirection)
            continue;

         double deltaAngle = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(desiredHeading, heading.getYawOffset()));
         if (Math.abs(deltaAngle) < secondClosestAngle)
         {
            secondClosestAngle = deltaAngle;
            secondaryDirection = heading;
         }
      }

      for (RobotSide stanceSide : RobotSide.values)
      {
         double stepLength0 = getIdealStepLength(parameters, primaryDirection);
         double stepWidth0 = getIdealStepWidth(parameters, primaryDirection, stanceSide);
         double stepLength1 = getIdealStepLength(parameters, secondaryDirection);
         double stepWidth1 = getIdealStepWidth(parameters, secondaryDirection, stanceSide);

         double thetaFromPrimary = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(desiredHeading, primaryDirection.getYawOffset()));
         double alpha0 = Math.cos(thetaFromPrimary);
         double alpha1 = Math.sin(thetaFromPrimary);

         idealStepLengths.get(stanceSide).set(alpha0 * stepLength0 + alpha1 * stepLength1);
         idealStepWidths.get(stanceSide).set(alpha0 * stepWidth0 + alpha1 * stepWidth1);
      }
   }

   private static double getIdealStepLength(FootstepPlannerParametersReadOnly parameters, FootstepPlanHeading heading)
   {
      switch (heading)
      {
         case LEFT:
         case RIGHT:
            return 0.0;
         case BACKWARD:
            return - parameters.getIdealBackStepLength();
         case FORWARD:
         default:
            return parameters.getIdealFootstepLength();
      }
   }

   private static double getIdealStepWidth(FootstepPlannerParametersReadOnly parameters, FootstepPlanHeading heading, RobotSide stanceSide)
   {
      switch (heading)
      {
         case LEFT:
            return stanceSide == RobotSide.LEFT ? - parameters.getMaximumStepWidth() : parameters.getMinimumStepWidth();
         case RIGHT:
            return stanceSide == RobotSide.LEFT ? - parameters.getMinimumStepWidth() : parameters.getMaximumStepWidth();
         case BACKWARD:
         case FORWARD:
         default:
            return stanceSide.negateIfLeftSide(parameters.getIdealSideStepWidth());
      }
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

      // If goal node is reachable, it's the ideal step
      FootstepNode goalNode = goalNodes.get(stanceNode.getRobotSide().getOppositeSide());
      if (!flatGroundMode() && nodeChecker.test(goalNode, stanceNode) && !attemptSquareUp)
      {
         return goalNode;
      }

      // Square up if requested
      Point2D midFootPoint = stanceNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      if (attemptSquareUp)
      {
         double turnYaw = 0.0;
         return turnInPlaceStep(stanceNode, midFootPoint, stanceNode.getRobotSide(), 0.5 * parameters.getIdealFootstepWidth(), turnYaw);
      }

      RobotSide stanceSide = stanceNode.getRobotSide();
      double alphaMidFoot = bodyPathPlanHolder.getClosestPoint(midFootPoint, projectionPose);
      int segmentIndex = bodyPathPlanHolder.getSegmentIndexFromAlpha(alphaMidFoot);
      bodyPathPlanHolder.getPointAlongPath(alphaMidFoot, projectionPose);

      double distanceFromGoalSquared = midFootPoint.distanceSquared(goalMidFootPose.getPosition());
      double distanceFromPathSquared = midFootPoint.distanceXYSquared(projectionPose.getPosition());
      double finalTurnProximity = parameters.getFinalTurnProximity();

      // Calculate target yaw:
      // 1) Match goal if close to goal
      // 2) Match body path if close to path
      // 3) Turn towards body path if off path

      double desiredYaw;
      if (distanceFromGoalSquared < MathTools.square(finalTurnProximity))
      {
         desiredYaw = goalMidFootPose.getYaw();
      }
      else if (distanceFromPathSquared < MathTools.square(parameters.getDistanceFromPathTolerance()))
      {
         desiredYaw = EuclidCoreTools.trimAngleMinusPiToPi(bodyPathPlanHolder.getSegmentYaw(segmentIndex) + desiredHeading);
      }
      else
      {
         int numberOfCorrectiveStepsWhenOffPath = 2;
         double alphaLookAhead = MathTools.clamp(alphaMidFoot + numberOfCorrectiveStepsWhenOffPath * parameters.getIdealFootstepLength() / pathLength, 0.0, 1.0);
         bodyPathPlanHolder.getPointAlongPath(alphaLookAhead, projectionPose);
         desiredYaw = Math.atan2(projectionPose.getY() - midFootPoint.getY(), projectionPose.getX() - midFootPoint.getX());
         desiredYaw = EuclidCoreTools.trimAngleMinusPiToPi(desiredYaw + desiredHeading);
      }

      // Clamp target yaw to what's achieveable by step parameters

      double deltaYaw = AngleTools.computeAngleDifferenceMinusPiToPi(desiredYaw, stanceNode.getYaw());
      RobotSide stepSide = stanceSide.getOppositeSide();
      double yawLowerLimit = stepSide == RobotSide.LEFT ? parameters.getMinimumStepYaw() : - parameters.getMaximumStepYaw();
      double yawUpperLimit = stepSide == RobotSide.LEFT ? parameters.getMaximumStepYaw() : - parameters.getMinimumStepYaw();
      double achievableStepYaw = MathTools.clamp(deltaYaw, yawLowerLimit, yawUpperLimit);

      // Calculate step positions:
      // 1) If close to goal, match the goal's mid-foot position and turn towards goal orientation
      // 2) If facing too far from body path orientation, turn in place to match body path heading
      //          (this avoids the blended step and turn that is unnatural if yaw difference is too large)
      // 3) Otherwise take a step given the ideal step parameters. Append a correction so that ideal step shifts towards path

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
         double idealStepLength = idealStepLengths.get(stanceSide).getDoubleValue();
         double idealStepWidth = idealStepWidths.get(stanceSide).getDoubleValue();

         stanceFootPose.set(stanceNode.getX(), stanceNode.getY(), stanceNode.getYaw());
         idealStep.set(stanceFootPose);
         idealStep.appendTranslation(idealStepLength, idealStepWidth);

         idealMidFootPose.set(idealStep);
         idealMidFootPose.appendTranslation(0.0, stepSide.negateIfLeftSide(0.5 * parameters.getIdealFootstepWidth()));
         calculateCorrectiveValues(idealMidFootPose);

         idealStep.getPosition().add(correctiveDistanceX.getDoubleValue(), correctiveDistanceY.getDoubleValue());
         idealStep.appendRotation(correctiveYaw.getDoubleValue());

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

   private void calculateCorrectiveValues(Pose2D midFootPose)
   {
      double alpha = bodyPathPlanHolder.getClosestPoint(midFootPose.getPosition(), projectionPose);

      Vector2D toClosestPointOnPath = new Vector2D();
      toClosestPointOnPath.set(projectionPose.getX(), projectionPose.getY());
      toClosestPointOnPath.sub(midFootPose.getX(), midFootPose.getY());

      double distanceToPath = toClosestPointOnPath.length();
      if (distanceToPath > maxDistanceAdjustmentTowardsPath)
      {
         toClosestPointOnPath.scale(maxDistanceAdjustmentTowardsPath / distanceToPath);
      }

      correctiveDistanceX.set(toClosestPointOnPath.getX());
      correctiveDistanceY.set(toClosestPointOnPath.getY());

      double pathYaw = bodyPathPlanHolder.getSegmentYaw(bodyPathPlanHolder.getSegmentIndexFromAlpha(alpha)) + desiredHeading;
      double currentYaw = midFootPose.getYaw();
      correctiveYaw.set(MathTools.clamp(AngleTools.computeAngleDifferenceMinusPiToPi(pathYaw, currentYaw), maxYawAdjustmentTowardsPath));
   }
}