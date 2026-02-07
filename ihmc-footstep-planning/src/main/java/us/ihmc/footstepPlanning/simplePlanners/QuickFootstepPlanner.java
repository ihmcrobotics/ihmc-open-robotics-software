package us.ihmc.footstepPlanning.simplePlanners;

import org.apache.commons.math3.util.Pair;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.Location;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

/**
 * A footstep planner that solves instantly, planning one step at a time.
 * The planner simply takes stance and goal footholds and plans the next step,
 * repeating until the goal is reached.
 */
public class QuickFootstepPlanner
{
   private int stepIndex;
   private SideDependentList<Pose3D> stances;
   private SideDependentList<Pose3D> goals;
   private final Pose3D swingEnd = new Pose3D();
   private RobotSide footToSwing = RobotSide.LEFT;
   private final Line3D stanceToGoalLine = new Line3D();
   private final Vector3D directionToGoal = new Vector3D();
   private final Pose3D stanceMid = new Pose3D();
   private final Pose3D goalMid = new Pose3D();
   private final Point3D approachGoalMid = new Point3D();
   private final SideDependentList<Point3D> midlineProjections = new SideDependentList<>(side -> new Point3D());
   private final Point3D midlinePoint = new Point3D();
   private final Vector3D oppositeStanceToProjection = new Vector3D();
   private boolean transistionToGoal;
   private Runnable stepPlannedCallback = () -> {};
   private double sidewaysness;

   public List<Pair<RobotSide, Pose3D>> plan(SideDependentList<Pose3D> stances, SideDependentList<Pose3D> goals)
   {
      List<Pair<RobotSide, Pose3D>> footstepPlan = new ArrayList<>();
      var currentStances = new SideDependentList<>(side -> new Pose3D(stances.get(side)));
      for (stepIndex = 0; stepIndex < 50; stepIndex++)
      {
         if (!planStep(currentStances, goals))
         {
            footstepPlan.add(new Pair<>(footToSwing, new Pose3D(swingEnd)));
            currentStances.get(footToSwing).set(swingEnd);
            stepPlannedCallback.run();
         }
         else
            break;
      }

      return footstepPlan;
   }

   public boolean planStep(SideDependentList<Pose3D> stances, SideDependentList<Pose3D> goals)
   {
      this.stances = stances;
      this.goals = goals;

      var atGoals = new SideDependentList<>(side -> stances.get(side).getPosition().distance(goals.get(side).getPosition()) <= 0.01
                                                 && stances.get(side).getOrientation().distance(goals.get(side).getOrientation()) <= Math.toRadians(5.0));
      if (atGoals.get(RobotSide.LEFT) && atGoals.get(RobotSide.RIGHT))
         return true;

      double stepLength = 0.4;
      double stepYaw = Math.toRadians(35.0);
//      for (RobotSide side : RobotSide.values) // Step directly to goals if possible
//      {
//         if (!atGoals.get(side)) // One foot might already be at the goal
//         {
//            // Avoid erroring out if goal feet impose a more aggressive step
//            double allowedLength = Math.max(stepLength, goals.get(side).getPosition().distance(goals.get(side.getOppositeSide()).getPosition()));
//            double allowedYaw = Math.max(stepYaw, Math.abs(goals.get(side).getOrientation().distance(stances.get(side.getOppositeSide()).getOrientation())));
//
//            boolean stepDirectlyToGoal = stances.get(side.getOppositeSide()).getPosition().distance(goals.get(side).getPosition()) <= allowedLength
//                      && Math.abs(goals.get(side).getOrientation().distance(stances.get(side.getOppositeSide()).getOrientation())) <= allowedYaw;
//
//            if (!atGoals.get(side.getOppositeSide())) // Skip crossover check if the goals are a crossover
//            {
//               // Disallow crossover steps
//               Vector3D oppositeStanceForward = new Vector3D(Axis3D.X);
//               stances.get(side.getOppositeSide()).getOrientation().transform(oppositeStanceForward);
//               Location location = EuclidGeometryTools.whichSideOfLine2DIsPoint2DOn(goals.get(side).getPosition().getX(),
//                                                                                    goals.get(side).getPosition().getY(),
//                                                                                    stances.get(side.getOppositeSide()).getX(),
//                                                                                    stances.get(side.getOppositeSide()).getY(),
//                                                                                    oppositeStanceForward.getX(),
//                                                                                    oppositeStanceForward.getY());
//               stepDirectlyToGoal &= location != null;
//               stepDirectlyToGoal &= side == RobotSide.LEFT && location == Location.LEFT || side == RobotSide.RIGHT && location == Location.RIGHT;
//            }
//
//            // Need to disallow step if sideways and need to step closer foot
//
//            // Disallow step if it's within position of opposite stance foot
//
//            if (stepDirectlyToGoal)
//            {
//               footToSwing = side;
//               swingEnd.set(goals.get(side));
//               return false;
//            }
//         }
//      }

      stanceMid.interpolate(stances.get(RobotSide.LEFT), stances.get(RobotSide.RIGHT), 0.5);
      goalMid.interpolate(goals.get(RobotSide.LEFT), goals.get(RobotSide.RIGHT), 0.5);

      // Compute sidewaysness: 1 straight sideways, 0.5 diagonal, 0 forward/backward
      Vector3D stanceToGoal = new Vector3D();
      stanceToGoal.sub(goalMid.getPosition(), stanceMid.getPosition());
      stanceToGoal.normalize();
      Vector3D stanceMidForward = new Vector3D(Axis3D.X);
      stanceMid.getOrientation().transform(stanceMidForward);
      sidewaysness = 1.0 - (2.0 / Math.PI) * Math.abs(Math.asin(stanceMidForward.dot(stanceToGoal)));

      Vector2D bisectorDirection = new Vector2D();
      EuclidGeometryTools.perpendicularBisector2D(new Point2D(goals.get(RobotSide.RIGHT).getPosition()),
                                                  new Point2D(goals.get(RobotSide.LEFT).getPosition()),
                                                  new Point2D(), // Bisector start
                                                  bisectorDirection);
      Vector3D midFeetForward3D = new Vector3D(Axis3D.X);
      goalMid.getOrientation().transform(midFeetForward3D);
      Vector2D midFeetForward = new Vector2D(midFeetForward3D.getX(), midFeetForward3D.getY());
      if (midFeetForward.dot(bisectorDirection) > 0.0)
         bisectorDirection.negate();
      Vector3D stanceForward3D = new Vector3D(Axis3D.X);
      stanceMid.getOrientation().transform(stanceForward3D);
      Vector3D stanceToGoalMid = new Vector3D();
      stanceToGoalMid.sub(goalMid.getPosition(), stanceMid.getPosition());
      if (stanceForward3D.dot(stanceToGoalMid) < 0.0)
         bisectorDirection.negate();
      approachGoalMid.scaleAdd(stepLength * 0.3 * (1.0 - sidewaysness), new Vector3D(bisectorDirection), goalMid.getPosition());

      stanceToGoalLine.set(stanceMid.getPosition(), approachGoalMid);
      directionToGoal.sub(approachGoalMid, stanceMid.getPosition());
      directionToGoal.normalize();

      for (RobotSide side : RobotSide.values)
         midlineProjections.get(side).set(stanceToGoalLine.orthogonalProjectionCopy(stances.get(side).getPosition()));
      SideDependentList<Double> distancesToGoalMid = new SideDependentList<>(side -> midlineProjections.get(side).distance(approachGoalMid));

      for (RobotSide side : RobotSide.values) // Take a step towards the goal
      {
         if (atGoals.get(side)) // Never step a foot already at goal
            continue;
         if (atGoals.get(side.getOppositeSide())) // Always take last goal step
         {
            footToSwing = side;
            swingEnd.set(goals.get(side));
            return false;
         }

         // Choose swing foot -- furthest foot from goal
         boolean isFurthest = distancesToGoalMid.get(side) >= distancesToGoalMid.get(side.getOppositeSide());
         double stanceDistance = midlineProjections.get(side).distance(midlineProjections.get(side.getOppositeSide()));

         // When walking sideways, if feet are close together, swing foot closest to goal instead TODO: Make sidewaysness activation > 0.5
         if (sidewaysness > 0.5 ? (stanceDistance < 0.3) != isFurthest : isFurthest)
         {
            footToSwing = side;

            if (sidewaysness > 0.5 && isFurthest) // Swing up to stance foot
            {
               double x0 = 0.5, y0 = 0.12; // Diagonal
               double x1 = 1.0, y1 = 0.15; // Straight sideways
               double behindStanceFoot = y0 + (sidewaysness - x0) * (y1 - y0) / (x1 - x0);
               midlinePoint.scaleAdd(-behindStanceFoot, directionToGoal, midlineProjections.get(side.getOppositeSide()));
            }
            else // Normal -- swing past stance foot
            {
               // Avoid erroring out if goal feet impose a more aggressive step
               double allowedLength = Math.max(stepLength, goals.get(side).getPosition().distance(goals.get(side.getOppositeSide()).getPosition()));
               double allowedYaw = Math.max(stepYaw, Math.abs(goals.get(side).getOrientation().distance(stances.get(side.getOppositeSide()).getOrientation())));
               if (stances.get(side.getOppositeSide()).getPosition().distance(goals.get(side).getPosition()) <= allowedLength
                  && Math.abs(goals.get(side).getOrientation().distance(stances.get(side.getOppositeSide()).getOrientation())) <= allowedYaw)
               { // Step directly to goal
                  swingEnd.set(goals.get(side)); // TODO: Avoid stepping on stance foot
                  return false;
               }
               else
               {
                  double stepAlongMidline = stepLength * (0.6 + 0.3 * sidewaysness);
                  // Scale down step length as we approach goal to avoid small final steps
                  transistionToGoal = distancesToGoalMid.get(side.getOppositeSide()) < 2.0 * stepAlongMidline;
   //               if (transistionToGoal)
   //                  stepAlongMidline = oppositeStanceMidlineProjection.distance(goalMid.getPosition()) / 2.0;
                  midlinePoint.scaleAdd(stepAlongMidline, directionToGoal, midlineProjections.get(side.getOppositeSide()));
               }
            }

            // TODO: Increase this offset to avoid stance foot
            double midlineOffset = (1.0 - sidewaysness) * 0.12;
            oppositeStanceToProjection.sub(midlineProjections.get(side.getOppositeSide()), stances.get(side.getOppositeSide()).getPosition());
            oppositeStanceToProjection.normalize();
            swingEnd.getPosition().scaleAdd(midlineOffset, oppositeStanceToProjection, midlinePoint);

            Quaternion swingEndOrientation = new Quaternion(stanceMid.getOrientation());
            if (midlinePoint.distance(approachGoalMid) < 1.0) // If close to goal, average in goal feet
               swingEndOrientation.interpolate(swingEndOrientation, goalMid.getOrientation(), 0.5);

            swingEnd.getOrientation().set(swingEndOrientation);
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
      return stances.get(footToSwing.getOppositeSide()).getPosition();
   }

   public SideDependentList<Point3D> getMidlineProjections()
   {
      return midlineProjections;
   }

   public Point3D getOppositeStanceMidlineProjection()
   {
      return midlineProjections.get(footToSwing.getOppositeSide());
   }

   public boolean getTransistionToGoal()
   {
      return transistionToGoal;
   }

   public double getSidewaysness()
   {
      return sidewaysness;
   }
}
