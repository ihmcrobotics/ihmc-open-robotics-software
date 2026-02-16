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
import us.ihmc.robotics.robotSide.SideMap;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

/**
 * A footstep planner that solves instantly, planning one step at a time.
 * The planner simply takes stance and goal footholds and plans the next step,
 * repeating until the goal is reached.
 */
public class QuickFootstepPlanner
{
   private int stepIndex;
   private int maxSteps = 50;
   private final SideMap<Pose3D> stance = new SideMap<>(() -> new Pose3D());
   private final SideMap<Pose3D> goal = new SideMap<>(() -> new Pose3D());
   private final Pose3D swingEnd = new Pose3D();
   private RobotSide footToSwing = RobotSide.LEFT;
   private final Line3D midline = new Line3D();
   private final Vector3D directionToGoal = new Vector3D();
   private final Pose3D stanceMid = new Pose3D();
   private final Pose3D goalMid = new Pose3D();
   private final SideMap<Point3D> stanceHip = new SideMap<>(() -> new Point3D());
   private final SideMap<Point3D> goalHip = new SideMap<>(() -> new Point3D());
   private final Point3D approachGoalMid = new Point3D();
   private final SideMap<Point3D> midlineProjection = new SideMap<>(side -> new Point3D());
   private final Point3D midlinePoint = new Point3D();
   private final Vector3D oppositeStanceToProjection = new Vector3D();
   private boolean transistionToGoal;
   private Runnable stepPlannedCallback = () -> {};
   private double sidewaysness;
   private boolean sidestep;

   public List<Pair<RobotSide, Pose3D>> plan(EnumMap<RobotSide, Pose3D> stance, EnumMap<RobotSide, Pose3D> goal)
   {
      for (RobotSide side : RobotSide.values)
      {
         this.goal.get(side).set(goal.get(side));
         this.stance.get(side).set(stance.get(side));
      }

      List<Pair<RobotSide, Pose3D>> footstepPlan = new ArrayList<>();
      for (stepIndex = 0; stepIndex < maxSteps; stepIndex++)
      {
         if (!planStep())
         {
            footstepPlan.add(new Pair<>(footToSwing, new Pose3D(swingEnd)));
            this.stance.get(footToSwing).set(swingEnd);
            stepPlannedCallback.run();
         }
         else
            break;
      }

      return footstepPlan;
   }

   private boolean planStep()
   {
      SideMap<Boolean> atGoal = new SideMap<>(side -> stance.get(side).getPosition().distance(goal.get(side).getPosition()) <= 0.01
                                                && stance.get(side).getOrientation().distance(goal.get(side).getOrientation()) <= Math.toRadians(5.0));
      if (atGoal.get(RobotSide.LEFT) && atGoal.get(RobotSide.RIGHT))
         return true;

      double stepLength = 0.4;
      double stepYaw = Math.toRadians(35.0);
      stanceMid.interpolate(stance.get(RobotSide.LEFT), stance.get(RobotSide.RIGHT), 0.5);
      goalMid.interpolate(goal.get(RobotSide.LEFT), goal.get(RobotSide.RIGHT), 0.5);

      double hipWidth = 0.12;
      double hipHeight = 0.8;
      for (RobotSide side : RobotSide.values)
      {
         stanceHip.get(side).set(0.0, side.negateIfRightSide(hipWidth), 0.0);
         stanceMid.transform(stanceHip.get(side));
         stanceHip.get(side).addZ(hipHeight);
         goalHip.get(side).set(0.0, side.negateIfRightSide(hipWidth), 0.0);
         goalMid.transform(goalHip.get(side));
         goalHip.get(side).addZ(hipHeight);
      }

      // Compute sidewaysness: 1 straight sideways, 0.5 diagonal, 0 forward/backward
      Vector3D stanceToGoal = new Vector3D();
      stanceToGoal.sub(goalMid.getPosition(), stanceMid.getPosition());
      stanceToGoal.normalize();
      Vector3D stanceMidForward = new Vector3D(Axis3D.X);
      stanceMid.getOrientation().transform(stanceMidForward);
      sidewaysness = 1.0 - (2.0 / Math.PI) * Math.abs(Math.asin(stanceMidForward.dot(stanceToGoal)));
      sidestep = sidewaysness > 0.7;
      sidestep = false;
      // TODO: Sidewaysness can get weird at the end

//      if (sidestep)
      if (true)
         approachGoalMid.set(goalMid.getPosition());
      else
      {
         Vector2D bisectorDirection = new Vector2D();
         EuclidGeometryTools.perpendicularBisector2D(new Point2D(goal.get(RobotSide.RIGHT).getPosition()),
                                                     new Point2D(goal.get(RobotSide.LEFT).getPosition()),
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
         approachGoalMid.scaleAdd(stepLength * 0.3, new Vector3D(bisectorDirection), goalMid.getPosition());
      }

      midline.set(stanceMid.getPosition(), approachGoalMid);
      directionToGoal.sub(approachGoalMid, stanceMid.getPosition());
      directionToGoal.normalize();

      SideMap<Double> distanceToGoalMid = new SideMap<>();
      for (RobotSide side : RobotSide.values)
      {
         midlineProjection.get(side).set(midline.orthogonalProjectionCopy(stance.get(side).getPosition()));
         distanceToGoalMid.put(side, midlineProjection.get(side).distance(approachGoalMid));
      }

      SideMap<Boolean> isFurthest = new SideMap<>();
      SideMap<Double> stanceDistance = new SideMap<>();
      SideMap<Boolean> canReachGoal = new SideMap<>();
      SideMap<Boolean> goalIsCrossover = new SideMap<>();
      for (RobotSide side : RobotSide.values)
      {
         isFurthest.put(side, distanceToGoalMid.get(side) >= distanceToGoalMid.get(side.getOppositeSide()));
         stanceDistance.put(side, midlineProjection.get(side).distance(midlineProjection.get(side.getOppositeSide())));

         double allowedLength = Math.max(stepLength, goal.get(side).getPosition().distance(goal.get(side.getOppositeSide()).getPosition()));
         double allowedYaw = Math.max(stepYaw, Math.abs(goal.get(side).getOrientation().distance(stance.get(side.getOppositeSide()).getOrientation())));
         canReachGoal.put(side, stance.get(side.getOppositeSide()).getPosition().distance(goal.get(side).getPosition()) <= allowedLength
             && Math.abs(goal.get(side).getOrientation().distance(stance.get(side.getOppositeSide()).getOrientation())) <= allowedYaw);

         Vector3D oppositeStanceForward = new Vector3D(Axis3D.X);
         stance.get(side.getOppositeSide()).getOrientation().transform(oppositeStanceForward);
         Location location = EuclidGeometryTools.whichSideOfLine2DIsPoint2DOn(goal.get(side).getPosition().getX(),
                                                                              goal.get(side).getPosition().getY(),
                                                                              stance.get(side.getOppositeSide()).getX(),
                                                                              stance.get(side.getOppositeSide()).getY(),
                                                                              oppositeStanceForward.getX(),
                                                                              oppositeStanceForward.getY());
         goalIsCrossover.put(side, location == null || (side == RobotSide.LEFT && location == Location.RIGHT
                                                     || side == RobotSide.RIGHT && location == Location.LEFT));
      }

      // TODO: Calculate data for each foot before deciding which foot to swing

      // Evaluate criteria for swing side selection
      // TODO: Need "isBlocked" which is either crossover
      //   or stepping on opposite stance or goal foot
      //   In the case a footstep is blocked, we need to store the blocking Pose3D
      //     and compute a modified step location, probably rotating the prospective
      //     footstep about the stance foot away from the blocking step
      /**
       * If it's reachable and blocked by crossover, the step the blocking foot instead (don't perform crossover check for this foot)
       * If you're stepping not to the goal, check if you are stepping on the opposite goal foot
       */

      SideMap<Pose3D> candidate = new SideMap<>(() -> new Pose3D());
      SideMap<Boolean> overlapsOppositeStep = new SideMap<>();
      for (RobotSide side : RobotSide.values)
      {
         if (sidestep && isFurthest.get(side)) // Swing up to stance foot
         {
            double x0 = 0.5, y0 = 0.12; // Diagonal
            double x1 = 1.0, y1 = 0.15; // Straight sideways
            double behindStanceFoot = y0 + (sidewaysness - x0) * (y1 - y0) / (x1 - x0);
            midlinePoint.scaleAdd(-behindStanceFoot, directionToGoal, midlineProjection.get(side.getOppositeSide()));
         }
         else
         {
            double stepAlongMidline = stepLength * (0.6 + 0.3 * sidewaysness);
            // Scale down step length as we approach goal to avoid small final steps
            transistionToGoal = distanceToGoalMid.get(side.getOppositeSide()) < 2.0 * stepAlongMidline;
            //  if (transistionToGoal)
            //     stepAlongMidline = oppositeStanceMidlineProjection.distance(goalMid.getPosition()) / 2.0;
            midlinePoint.scaleAdd(stepAlongMidline, directionToGoal, midlineProjection.get(side.getOppositeSide()));
         }

         // TODO: Increase this offset to avoid stance foot
         double midlineOffset = (1.0 - sidewaysness) * 0.12;
         oppositeStanceToProjection.sub(midlineProjection.get(side.getOppositeSide()), stance.get(side.getOppositeSide()).getPosition());
         oppositeStanceToProjection.normalize();
         candidate.get(side).getPosition().scaleAdd(midlineOffset, oppositeStanceToProjection, midlinePoint);

         // TODO Check opposite goal and stance

         double minClosenessX = 0.15;
         double minClosenessY = 0.15;
         double distanceToOppositeStance = candidate.get(side).getPosition().distance(stance.get(side.getOppositeSide()).getPosition());
         if (distanceToOppositeStance < minClosenessX)
         {
            Vector3D toCandidate = new Vector3D();
            toCandidate.sub(candidate.get(side).getPosition(), stance.get(side.getOppositeSide()).getPosition());
            toCandidate.normalize();
            candidate.get(side).getPosition().scaleAdd(minClosenessX, toCandidate, stance.get(side.getOppositeSide()).getPosition());
         }


         double distanceToOppositeGoal = candidate.get(side).getPosition().distance(goal.get(side.getOppositeSide()).getPosition());
         if (distanceToOppositeGoal < minClosenessX)
         {
            Vector3D toCandidate = new Vector3D();
            toCandidate.sub(candidate.get(side).getPosition(), goal.get(side.getOppositeSide()).getPosition());
            toCandidate.normalize();
            candidate.get(side).getPosition().scaleAdd(minClosenessX, toCandidate, goal.get(side.getOppositeSide()).getPosition());
         }

      }

      for (RobotSide side : RobotSide.values) // Take a step towards the goal
      {
         if (atGoal.get(side)) // Never step a foot already at goal
            continue;
         if (atGoal.get(side.getOppositeSide())) // Always take last goal step
         {
            footToSwing = side;
            swingEnd.set(goal.get(side));
            return false;
         }

         // Choose swing foot -- furthest foot from goal

         // When walking sideways, if feet are close together, swing foot closest to goal instead
         if (sidestep ? (stanceDistance.get(side) < 0.3) != isFurthest.get(side) : isFurthest.get(side))
         {
            footToSwing = side;

            if (!sidestep || !isFurthest.get(side))
            {
               boolean stepDirectlyToGoal = canReachGoal.get(side) && !goalIsCrossover.get(side);
               // TODO: Avoid stepping on stance foot
               if (stepDirectlyToGoal)
               {
                  swingEnd.set(goal.get(side));
                  return false;
               }
            }

            swingEnd.getPosition().set(candidate.get(side).getPosition());

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

   public void setMaxSteps(int maxSteps)
   {
      this.maxSteps = maxSteps;
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

   public SideMap<Point3D> getStanceHip()
   {
      return stanceHip;
   }

   public SideMap<Point3D> getGoalHip()
   {
      return goalHip;
   }

   public Point3D getApproachGoalMid()
   {
      return approachGoalMid;
   }

   public Point3D getOppositeStance()
   {
      return stance.get(footToSwing.getOppositeSide()).getPosition();
   }

   public SideMap<Point3D> getMidlineProjection()
   {
      return midlineProjection;
   }

   public Point3D getOppositeStanceMidlineProjection()
   {
      return midlineProjection.get(footToSwing.getOppositeSide());
   }

   public boolean getTransistionToGoal()
   {
      return transistionToGoal;
   }

   public double getSidewaysness()
   {
      return sidewaysness;
   }

   public boolean isSidestep()
   {
      return sidestep;
   }
}
