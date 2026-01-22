package us.ihmc.footstepPlanning.simplePlanners;

import org.apache.commons.math3.util.Pair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

/**
 * A footstep planner that solves instantly, planning one step at a time.
 * The planner simply takes stance and goal footholds and plans the next step,
 * repeating until the goal is reached.
 */
public class QuickFootstepPlanner
{
   private final Pose3D swingEnd = new Pose3D();
   private RobotSide footToSwing = RobotSide.LEFT;
   private final Line3D stanceToGoalLine = new Line3D();
   private final Vector3D directionToGoal = new Vector3D();
   private final Pose3D stanceMid = new Pose3D();
   private final Pose3D goalMid = new Pose3D();
   private final Point3D approachGoalMid = new Point3D();
   private final Point3D oppositeStance = new Point3D();
   private final Point3D oppositeStanceMidlineProjection = new Point3D();
   private final Point3D midlinePoint = new Point3D();
   private final Vector3D oppositeStanceToProjection = new Vector3D();
   private boolean transistionToGoal;
   private Runnable stepPlannedCallback = () -> {};

   public List<Pair<RobotSide, Pose3D>> plan(SideDependentList<Pose3D> stances, SideDependentList<Pose3D> goals)
   {
      List<Pair<RobotSide, Pose3D>> footstepPlan = new java.util.ArrayList<>();

      SideDependentList<Pose3D> currentStances = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         currentStances.put(side, new Pose3D(stances.get(side)));
      }

      boolean reachedGoal = false;
      int maxIterations = 100;
      int iteration = 0;

      while (!reachedGoal && iteration < maxIterations)
      {
         reachedGoal = planStep(currentStances, goals);

         if (!reachedGoal)
         {
            footstepPlan.add(new Pair<>(footToSwing, new Pose3D(swingEnd)));
            currentStances.get(footToSwing).set(swingEnd);
            stepPlannedCallback.run();
         }

         iteration++;
      }

      return footstepPlan;
   }

   public boolean planStep(SideDependentList<Pose3D> stances, SideDependentList<Pose3D> goals)
   {
      double positionThreshold = 0.01;
      double orientationThreshold = Math.toRadians(5.0);

      SideDependentList<Double> positionErrors
            = new SideDependentList<>(side -> stances.get(side).getPosition().distance(goals.get(side).getPosition()));
      SideDependentList<Double> orientationErrors
            = new SideDependentList<>(side -> stances.get(side).getOrientation().distance(goals.get(side).getOrientation()));
      SideDependentList<Boolean> atGoals = new SideDependentList<>(side -> positionErrors.get(side) <= positionThreshold
                                                                        && orientationErrors.get(side) <= orientationThreshold);

      if (atGoals.get(RobotSide.LEFT) && atGoals.get(RobotSide.RIGHT))
         return true;

      double idealStepLength = 0.4;
      double idealStepYaw = Math.toRadians(35.0);
      for (RobotSide side : RobotSide.values) // Step directly to goals if possible
      {
         if (!atGoals.get(side))
         {
            double oppositeStanceDistanceToGoal = stances.get(side.getOppositeSide()).getPosition().distance(goals.get(side).getPosition());
            double oppositeStanceYawToGoal = Math.abs(goals.get(side).getOrientation().getYaw() - stances.get(side.getOppositeSide()).getOrientation().getYaw());

            // Avoid erroring out if goal feet are farther apart than the step length
            double allowedLength = Math.max(idealStepLength, goals.get(side).getPosition().distance(goals.get(side.getOppositeSide()).getPosition()));
            // Avoid erroring out if goal feet are more yawed than the step yaw
            double allowedYaw = Math.max(idealStepYaw,
                                         Math.abs(goals.get(side).getOrientation().getYaw() - stances.get(side.getOppositeSide()).getOrientation().getYaw()));
            if (oppositeStanceDistanceToGoal <= allowedLength && oppositeStanceYawToGoal <= allowedYaw)
            {
               footToSwing = side;
               swingEnd.set(goals.get(side));
               return false;
            }
         }
      }

      SideDependentList<Double> distancesToGoalMid = new SideDependentList<>(side -> stances.get(side).getPosition().distance(goalMid.getPosition()));
      for (RobotSide side : RobotSide.values) // Take a step towards the goal
      {
         if (distancesToGoalMid.get(side) >= distancesToGoalMid.get(side.getOppositeSide())) // Choose swing foot; furthest from goal
         {
            footToSwing = side;

            double stepAlongMidline = idealStepLength * 0.6;

            stanceMid.interpolate(stances.get(side), stances.get(side.getOppositeSide()), 0.5);
            goalMid.interpolate(goals.get(side), goals.get(side.getOppositeSide()), 0.5);

            Point2D bisectorStart = new Point2D();
            Vector2D bisectorDirection = new Vector2D();
            EuclidGeometryTools.perpendicularBisector2D(new Point2D(goals.get(RobotSide.RIGHT).getPosition()),
                                                        new Point2D(goals.get(RobotSide.LEFT).getPosition()),
                                                        bisectorStart,
                                                        bisectorDirection);
            Vector3D midFeetForward3D = new Vector3D(1.0, 0.0, 0.0);
            goalMid.getOrientation().transform(midFeetForward3D);
            Vector2D midFeetForward = new Vector2D(midFeetForward3D.getX(), midFeetForward3D.getY());
            if (midFeetForward.dot(bisectorDirection) > 0.0)
               bisectorDirection.negate();
            Vector3D stanceForward3D = new Vector3D(1.0, 0.0, 0.0);
            stanceMid.getOrientation().transform(stanceForward3D);
            Vector3D stanceToGoalMid = new Vector3D();
            stanceToGoalMid.sub(goalMid.getPosition(), stanceMid.getPosition());
            if (stanceForward3D.dot(stanceToGoalMid) < 0.0)
               bisectorDirection.negate();

            approachGoalMid.scaleAdd(idealStepLength * 0.3, new Vector3D(bisectorDirection), goalMid.getPosition());

            stanceToGoalLine.set(stanceMid.getPosition(), approachGoalMid);
            directionToGoal.sub(approachGoalMid, stanceMid.getPosition());
            directionToGoal.normalize();

            oppositeStance.set(stances.get(side.getOppositeSide()).getPosition());
            oppositeStanceMidlineProjection.set(stanceToGoalLine.orthogonalProjectionCopy(oppositeStance));

            // Scale down step length as we approach goal to avoid small final steps
            double transitionDistance = 2.0 * idealStepLength * 0.6;
            transistionToGoal = oppositeStanceMidlineProjection.distance(approachGoalMid) < transitionDistance;
            if (transistionToGoal)
               stepAlongMidline = oppositeStanceMidlineProjection.distance(approachGoalMid) / 2.0;

            midlinePoint.scaleAdd(stepAlongMidline, directionToGoal, oppositeStanceMidlineProjection);

            oppositeStanceToProjection.sub(oppositeStanceMidlineProjection, stances.get(side.getOppositeSide()).getPosition());
            oppositeStanceToProjection.normalize();

            double targetDistanceFromLine = 0.12;
            swingEnd.getPosition().scaleAdd(targetDistanceFromLine, oppositeStanceToProjection, midlinePoint);

            double yawSum = 0.0;
            int yawWeight = 0;
            for (RobotSide side2 : RobotSide.values) // Always average in stance feet
            {
               yawSum += stances.get(side2).getOrientation().getYaw();
               ++yawWeight;
            }
            if (midlinePoint.distance(approachGoalMid) < 1.0) // If close to goal, average in goal feet
            {
               for (RobotSide side2 : RobotSide.values)
               {
                  yawSum += goals.get(side2).getOrientation().getYaw();
                  ++yawWeight;
               }
            }
            else // If far from goal, average in yaw along line
            {
               double yawAlongLine = Math.atan2(stanceToGoalLine.getDirection().getY(), stanceToGoalLine.getDirection().getX());
               if (Math.abs(yawAlongLine) > Math.PI / 2.0)
                  yawAlongLine -= Math.copySign(Math.PI, yawAlongLine);
               yawSum += yawAlongLine;
               ++yawWeight;
            }

            double yaw = yawSum / yawWeight;

            double yawAmount = EuclidCoreTools.angleDifferenceMinusPiToPi(yaw, stances.get(side.getOppositeSide()).getOrientation().getYaw());
            if (yawAmount > idealStepYaw) // Clamp yaw amount
               yaw = stances.get(side.getOppositeSide()).getOrientation().getYaw() + Math.copySign(idealStepYaw, yawAmount);

            swingEnd.getOrientation().setToYawOrientation(yaw);
         }
      }

      return false;
   }

   public void setStepPlannedCallback(Runnable stepPlannedCallback)
   {
      this.stepPlannedCallback = stepPlannedCallback;
   }

   public RobotSide getFootToSwing()
   {
      return footToSwing;
   }

   public Pose3D getSwingEnd()
   {
      return swingEnd;
   }

   public Pose3D getStanceMid()
   {
      return stanceMid;
   }

   public Pose3D getGoalMid()
   {
      return goalMid;
   }

   public Point3D getApproachGoalMid()
   {
      return approachGoalMid;
   }

   public Point3D getOppositeStance()
   {
      return oppositeStance;
   }

   public Point3D getOppositeStanceMidlineProjection()
   {
      return oppositeStanceMidlineProjection;
   }

   public boolean getTransistionToGoal()
   {
      return transistionToGoal;
   }
}
