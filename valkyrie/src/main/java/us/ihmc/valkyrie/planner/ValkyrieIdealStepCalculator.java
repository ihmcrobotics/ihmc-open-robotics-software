package us.ihmc.valkyrie.planner;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.valkyrie.planner.BodyPathHelper.WaypointData;

import java.util.HashMap;

public class ValkyrieIdealStepCalculator
{
   private final ValkyrieAStarFootstepPlannerParameters parameters;
   private final HashMap<FootstepNode, FootstepNode> idealStepMap = new HashMap<>();
   private final BodyPathHelper bodyPathHelper;
   private final ValkyrieFootstepValidityChecker stepChecker;

   public ValkyrieIdealStepCalculator(ValkyrieAStarFootstepPlannerParameters parameters,
                                      BodyPathHelper bodyPathHelper,
                                      ValkyrieFootstepValidityChecker stepChecker)
   {
      this.parameters = parameters;
      this.bodyPathHelper = bodyPathHelper;
      this.stepChecker = stepChecker;
   }

   public void initialize()
   {
      idealStepMap.clear();
   }

   public FootstepNode computeIdealStep(FootstepNode stanceNode)
   {
      return idealStepMap.computeIfAbsent(stanceNode, this::computeIdealStepInternal);
   }

   private FootstepNode computeIdealStepInternal(FootstepNode stanceNode)
   {
      double lookAheadDistance = parameters.getIdealFootstepLength();

      WaypointData waypointData = bodyPathHelper.getWaypointFromNode(stanceNode, lookAheadDistance);
      double pathHeading = waypointData.getPathHeading();
      Line2D pathLine = waypointData.getPathLine();
      SideDependentList<FootstepNode> goalNodes = waypointData.getGoalNodes();
      Pose2DBasics goalMidFootPose = waypointData.getNominalGoalPose();

      FootstepNode goalNode = goalNodes.get(stanceNode.getRobotSide().getOppositeSide());
      if (stepChecker.checkFootstep(goalNode, stanceNode))
      {
         // goal node is reachable
         return goalNode;
      }

      Point2D midFootPoint = stanceNode.getOrComputeMidFootPoint(parameters.getIdealFootstepWidth());
      RobotSide stanceSide = stanceNode.getRobotSide();

      double distanceFromGoalSquared = midFootPoint.distanceSquared(goalMidFootPose.getPosition());
      double distanceFromPath = pathLine.distance(midFootPoint);
      double desiredYaw;

      double finalTurnProximity = parameters.getFinalTurnProximity();
      double distanceToleranceToMatchHeading = 0.25;
      double yawToleranceToTurnTowardsGoal = 0.35;

      if (distanceFromGoalSquared < MathTools.square(finalTurnProximity))
      {
         desiredYaw = goalMidFootPose.getYaw();
      }
      else if (distanceFromPath < distanceToleranceToMatchHeading)
      {
         desiredYaw = pathHeading;
      }
      else
      {
         desiredYaw = Math.atan2(goalMidFootPose.getY() - midFootPoint.getY(), goalMidFootPose.getX() - midFootPoint.getX());
      }

      double deltaYaw = AngleTools.computeAngleDifferenceMinusPiToPi(stanceNode.getYaw(), desiredYaw);
      double turnYaw;

      if (deltaYaw > 0.0)
      {
         turnYaw = stanceSide == RobotSide.LEFT ? - parameters.getMaximumStepYaw() : parameters.getMinimumStepYaw();
         turnYaw = Math.max(-deltaYaw, turnYaw);
      }
      else
      {
         turnYaw = stanceSide == RobotSide.LEFT ? - parameters.getMinimumStepYaw() : parameters.getMaximumStepYaw();
         turnYaw = Math.min(deltaYaw, turnYaw);
      }

      if (distanceFromGoalSquared <= MathTools.square(finalTurnProximity))
      {
         // turn in place at goal
         return turnInPlaceStep(stanceNode, goalMidFootPose.getPosition(), stanceSide, parameters.getIdealFootstepWidth(), turnYaw);
      }
      else if (Math.abs(deltaYaw) > yawToleranceToTurnTowardsGoal)
      {
         // turn in place towards goal
         return turnInPlaceStep(stanceNode, midFootPoint, stanceSide, parameters.getIdealFootstepWidth(), turnYaw);
      }
      else
      {
         // do ideal step with turn
         Pose2D idealStep = new Pose2D(midFootPoint, stanceNode.getYaw());
         idealStep.appendTranslation(parameters.getIdealFootstepLength(), stanceSide.negateIfLeftSide(parameters.getIdealFootstepWidth()));
         idealStep.appendRotation(turnYaw);
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
