package us.ihmc.footstepPlanning.simplePlanners;

import org.apache.commons.math3.util.Pair;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.Location;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
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
   private final SideMap<Line3D> hipLine = new SideMap<>(() -> new Line3D());
   private final Pose3D stanceMid = new Pose3D();
   private final Pose3D goalMid = new Pose3D();
   private final SideMap<Point3D> swingHip = new SideMap<>(() -> new Point3D());
   private final SideMap<Point3D> goalHip = new SideMap<>(() -> new Point3D());
   private Runnable stepPlannedCallback = () -> {};
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

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

      double stepLength = 0.33;
      double stepYaw = Math.toRadians(35.0);
      stanceMid.interpolate(stance.get(RobotSide.LEFT), stance.get(RobotSide.RIGHT), 0.5);
      goalMid.interpolate(goal.get(RobotSide.LEFT), goal.get(RobotSide.RIGHT), 0.5);

      double hipWidth = 0.12;
      for (RobotSide side : RobotSide.values)
      {
         swingHip.get(side).set(0.0, side.negateIfRightSide(2.0 * hipWidth), 0.0);
         stance.get(side.getOppositeSide()).transform(swingHip.get(side));
         goalHip.get(side).set(0.0, side.negateIfRightSide(hipWidth), 0.0);
         goalMid.transform(goalHip.get(side));
         hipLine.get(side).set(swingHip.get(side), goalHip.get(side)); // Maybe not needed?
      }

      Quaternion swingEndOrientation = new Quaternion(stanceMid.getOrientation());
      swingEnd.getOrientation().set(swingEndOrientation);

      SideMap<Pose3D> candidate = new SideMap<>(() -> new Pose3D());
      SideMap<Boolean> goalstepPossible = new SideMap<>();
      for (RobotSide side : RobotSide.values)
      {
         candidate.get(side).set(stance.get(side));
         candidate.get(side).getOrientation().set(swingEndOrientation);

         outer: // Calculate some possible steps toward the goal
         for (double distance = 0.02; distance < stepLength + 0.05; distance += 0.02)
         {
            candidate.get(side).getPosition().scaleAdd(distance, hipLine.get(side).getDirection(), swingHip.get(side));

            boolean isCrossover = isCrossover(stance.get(side.getOppositeSide()), candidate.get(side), side);
            if (isCrossover || distance > stepLength) // Done, check collision
            {
               Vector3D stanceForward = new Vector3D(Axis3D.X);
               stanceMid.getOrientation().transform(stanceForward);
               double direction = stanceForward.dot(hipLine.get(side).getDirection()) >= 0.0 ? 1.0 : -1.0;

               double resolution = 16;
               for (int i = 0; i < resolution; i++) // Revolve away from collision about hip
               {
                  ConvexPolygon2D candidatePolygon = createFootPolygon(candidate.get(side), 0.0);
                  ConvexPolygon2D stancePolygon = createFootPolygon(stance.get(side.getOppositeSide()), 0.04);
                  ConvexPolygon2D oppositeGoalPolygon = createFootPolygon(goal.get(side.getOppositeSide()), 0.04);
                  if (convexPolygonTools.doPolygonsIntersect(candidatePolygon, stancePolygon)
                   || convexPolygonTools.doPolygonsIntersect(candidatePolygon, oppositeGoalPolygon))
                  {
                     Vector3D ray = new Vector3D();
                     ray.sub(candidate.get(side).getPosition(), swingHip.get(side));
                     AxisAngle axisAngle = new AxisAngle(Axis3D.Z, Math.PI / resolution * side.negateIfRightSide(direction));
                     axisAngle.transform(ray);
                     candidate.get(side).getPosition().add(swingHip.get(side), ray);
                  }
                  else
                     break outer;
               }
            }
         }

         // Calculate if direct step to goal is possible
         double allowedLength = Math.max(stepLength, goal.get(side).getPosition().distance(goal.get(side.getOppositeSide()).getPosition()));
         goalstepPossible.put(side, false);
         Pose3D goalStep = new Pose3D(goal.get(side));
         Line3D goalLine = new Line3D(swingHip.get(side), goal.get(side).getPosition());
         for (double distance = 0.02; distance < allowedLength; distance += 0.02)
         {
            goalStep.getPosition().scaleAdd(distance, goalLine.getDirection(), swingHip.get(side));

            if (isCrossover(stance.get(side.getOppositeSide()), goalStep, side))
               break;

            if (goalStep.getPosition().distance(goal.get(side).getPosition()) < 0.05)
            {
               ConvexPolygon2D goalStepPolygon = createFootPolygon(goalStep, 0.0);
               ConvexPolygon2D stancePolygon = createFootPolygon(stance.get(side.getOppositeSide()), 0.04);
               if (!convexPolygonTools.doPolygonsIntersect(goalStepPolygon, stancePolygon))
               {
                  goalstepPossible.put(side, true);
                  break;
               }
            }
         }
      }

      for (RobotSide side : RobotSide.values) // Take a step towards the goal
      {
         if (atGoal.get(side)) // Never step a foot already at goal
            continue;
         if (atGoal.get(side.getOppositeSide()) || goalstepPossible.get(side)) // Always take last goal step
         {
            footToSwing = side;
            swingEnd.set(goal.get(side));
            return false;
         }

      }

      // Step the candidate that's a bigger step
      footToSwing = RobotSide.LEFT;
      double distance = stance.get(footToSwing).getPosition().distance(candidate.get(footToSwing).getPosition());
      double oppositeDistance = stance.get(footToSwing.getOppositeSide()).getPosition().distance(candidate.get(footToSwing.getOppositeSide()).getPosition());
      if (oppositeDistance > distance)
         footToSwing = footToSwing.getOppositeSide();
      swingEnd.set(candidate.get(footToSwing));
      return false;
   }

   public ConvexPolygon2D createFootPolygon(Pose3D pose, double boundary)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      double halfLength = 0.1 + boundary;
      double halfWidth = 0.05 + boundary;
      polygon.addVertex(halfLength, -halfWidth);
      polygon.addVertex(halfLength, halfWidth);
      polygon.addVertex(-halfLength, halfWidth);
      polygon.addVertex(-halfLength, -halfWidth);
      polygon.update();
      polygon.applyTransform(pose, false);
      return polygon;
   }

   private boolean isCrossover(Pose3D stance, Pose3D step, RobotSide stepSide)
   {
      Vector3D stanceForward = new Vector3D(Axis3D.X);
      stance.getOrientation().transform(stanceForward);
      Location location = EuclidGeometryTools.whichSideOfLine2DIsPoint2DOn(step.getPosition().getX(),
                                                                           step.getPosition().getY(),
                                                                           stance.getX(),
                                                                           stance.getY(),
                                                                           stanceForward.getX(),
                                                                           stanceForward.getY());
      return location == null || (stepSide == RobotSide.LEFT && location == Location.RIGHT
                               || stepSide == RobotSide.RIGHT && location == Location.LEFT);
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

   public SideMap<Point3D> getSwingHip()
   {
      return swingHip;
   }

   public SideMap<Point3D> getGoalHip()
   {
      return goalHip;
   }

   public SideMap<Line3D> getHipLine()
   {
      return hipLine;
   }

   public Point3D getOppositeStance()
   {
      return stance.get(footToSwing.getOppositeSide()).getPosition();
   }
}
