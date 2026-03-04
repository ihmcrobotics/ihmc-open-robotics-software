package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.Location;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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
   private double hipWidth = 0.12;
   private double stepLength = 0.28; // 0.33
   private double nextPelvisYawLimit = Math.toRadians(35.0);
   private double inwardLimit = 10.0;
   private double outwardLimit = 45.0; // 60
   private double stepAngleLimit = Math.toRadians(100); // 110
   private final Pose3D waypoint = new Pose3D();
   private final SideDependentList<Pose3D> stance = new SideDependentList<>(() -> new Pose3D());
   private final SideDependentList<Pose3D> goal = new SideDependentList<>(() -> new Pose3D());
   private final Pose3D swingEnd = new Pose3D();
   private RobotSide footToSwing = RobotSide.LEFT;
   private final SideDependentList<Vector3D> toGoalLinear = new SideDependentList<>(() -> new Vector3D());
   private final Pose3D goalMid = new Pose3D();
   private final SideDependentList<Pose3D> pelvis = new SideDependentList<>(() -> new Pose3D());
   private final SideDependentList<Pose3D> nextPelvis = new SideDependentList<>(() -> new Pose3D());
   private final SideDependentList<Pose3D> swingHip = new SideDependentList<>(() -> new Pose3D());
   private double swingDistance;
   private Runnable stepPlannedCallback = () -> {};
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
   private final Notification notification = new Notification();
   private boolean print;

   /** Plan to each waypoint and then to goal **/
   public List<QuickFootstep> plan(EnumMap<RobotSide, Pose3D> stance, List<Pose3D> waypoints, EnumMap<RobotSide, Pose3D> goal)
   {
      print = notification.poll();
      stepIndex = 0;
      for (RobotSide side : RobotSide.values)
         this.stance.get(side).set(stance.get(side));
      List<QuickFootstep> footstepPlan = new ArrayList<>();

      for (RobotSide side : RobotSide.values)
         this.goal.get(side).setToNaN();
      for (Pose3D waypoint : waypoints) // Plan to each waypoint without squaring up, facing waypoint X direction
      {
         this.waypoint.set(waypoint);
         footstepPlan.addAll(plan());
      }

      waypoint.setToNaN(); // Plan to exact goal footsteps
      if (goal != null) // Allow just planning to waypoint
      {
         for (RobotSide side : RobotSide.values)
            this.goal.get(side).set(goal.get(side));
         footstepPlan.addAll(plan());
      }
      return footstepPlan;
   }

   private List<QuickFootstep> plan()
   {
      List<QuickFootstep> footstepPlan = new ArrayList<>();
      for (; stepIndex < maxSteps; stepIndex++)
         if (!planStep())
         {
            footstepPlan.add(new QuickFootstep(footToSwing, swingEnd, swingDistance));
            this.stance.get(footToSwing).set(swingEnd);
            stepPlannedCallback.run();
         }
         else
            break;

      return footstepPlan;
   }

   private boolean planStep()
   {
      if (waypoint.containsNaN())
         goalMid.interpolate(goal.get(RobotSide.LEFT), goal.get(RobotSide.RIGHT), 0.5);
      else
         goalMid.set(waypoint);

      double goalPositionTolerance = 0.15;
      double goalOrientationTolerance = Math.toRadians(20.0);
      SideDependentList<Boolean> atGoal = new SideDependentList<>();
      if (waypoint.containsNaN())
      {
         for (RobotSide side : RobotSide.values)
               atGoal.put(side, stance.get(side).getPosition().distance(goal.get(side).getPosition()) <= 0.01
                             && stance.get(side).getOrientation().distance(goal.get(side).getOrientation()) <= Math.toRadians(5.0));
         if (atGoal.get(RobotSide.LEFT) && atGoal.get(RobotSide.RIGHT))
            return true;
      }

      SideDependentList<Pose3D> candidate = new SideDependentList<>(() -> new Pose3D());
      SideDependentList<Boolean> goalstepPossible = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         swingHip.get(side).set(0.0, side.negateIfRightSide(2.0 * hipWidth), 0.0, 0.0, 0.0, 0.0);
         stance.get(side.getOppositeSide()).transform(swingHip.get(side));
         pelvis.get(side).set(0.0, side.negateIfRightSide(hipWidth), 0.0, 0.0, 0.0, 0.0);
         stance.get(side.getOppositeSide()).transform(pelvis.get(side));

         if (!waypoint.containsNaN() // Check if we hit a waypoint
          && pelvis.get(side).getPosition().distance(goalMid.getPosition()) < goalPositionTolerance
          && pelvis.get(side).getOrientation().distance(goalMid.getOrientation()) < goalOrientationTolerance)
            return true;

         toGoalLinear.get(side).sub(goalMid.getPosition(), pelvis.get(side).getPosition());
         toGoalLinear.get(side).normalize();

         // Looking for a candidate hip
         // If close to goal, rotate to or toward goal orientation, else rotate to or toward path to goal
         nextPelvis.get(side).set(pelvis.get(side));

         if (pelvis.get(side).getPosition().distance(goalMid.getPosition()) > stepLength) // always translate pelvis to or toward goal
            nextPelvis.get(side).getPosition().scaleAdd(stepLength, toGoalLinear.get(side), nextPelvis.get(side).getPosition());
         else
            nextPelvis.get(side).getPosition().set(goalMid.getPosition());

         Quaternion toGoalFacingOrientation = new Quaternion();
         Vector3D toGoalFacing = new Vector3D(toGoalLinear.get(side));
         if (forward(pelvis.get(side)).dot(toGoalFacing) < 0.0) // Walk backwards to goal if it's behind us
            toGoalFacing.negate();
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.X, toGoalFacing, toGoalFacingOrientation);
         boolean inGoalRange = pelvis.get(side).getPosition().distance(goalMid.getPosition()) < 2.0 * stepLength;
         Quaternion desiredOrientation = new Quaternion(inGoalRange ? goalMid.getOrientation() : toGoalFacingOrientation);
         if (pelvis.get(side).getOrientation().distance(desiredOrientation) > nextPelvisYawLimit)
            new AxisAngle(Axis3D.Z, Math.signum(cross(forward(pelvis.get(side)),
                                                      forward(desiredOrientation)).getZ()) * nextPelvisYawLimit).transform(nextPelvis.get(side).getOrientation());
         else
            nextPelvis.get(side).getOrientation().set(desiredOrientation);

         // candidate hip
         Pose3D candidateHip = new Pose3D(0.0, side.negateIfRightSide(hipWidth), 0.0, 0.0, 0.0, 0.0);
         nextPelvis.get(side).transform(candidateHip);

         candidate.get(side).set(swingHip.get(side)); // Plan from the hip

         // Get step yaw calculated and clamped, so we can work with it, pretty much always take the max yaw
         Vector3D swingHipForward = forward(swingHip.get(side));
         Vector3D candidateHipForward = forward(candidateHip);
         double footYaw = yaw(swingHipForward, candidateHipForward);

         double footYawMin = side == RobotSide.LEFT ? Math.toRadians(-inwardLimit) : Math.toRadians(-outwardLimit);
         double footYawMax = side == RobotSide.LEFT ? Math.toRadians(outwardLimit) : Math.toRadians(inwardLimit);
         footYaw = MathTools.clamp(footYaw, footYawMin, footYawMax);

         Vector3D swingHipLateral = new Vector3D(0.0, side.negateIfRightSide(1.0), 0.0);
         swingHip.get(side).transform(swingHipLateral);
         Vector3D toCandidateHip = sub(candidateHip.getPosition(), swingHip.get(side).getPosition());
         double stepAngle = yaw(swingHipLateral, toCandidateHip);

         // crossovers naturally limited by stepAngle min/max
         double length;
         if (stepAngle < -stepAngleLimit || stepAngle > stepAngleLimit) // can't step in the direction we want
            length = 0.03; // step a tiny bit to inside of hip / penalize taking a step with this side
         else
            length = Math.min(stepLength, swingHip.get(side).getPosition().distance(candidateHip.getPosition()));

         // compute candidate pose from the 3 values
         double adjustedAngle = stepAngle + (side == RobotSide.LEFT ? Math.PI : 0.0);
         Vector3D toStepPosition = new Vector3D(length * Math.sin(adjustedAngle), length * -Math.cos(adjustedAngle), 0.0);
         swingHip.get(side).transform(toStepPosition);
         candidate.get(side).getPosition().add(toStepPosition);
         new AxisAngle(Axis3D.Z, footYaw).transform(candidate.get(side).getOrientation());

         if (waypoint.containsNaN())
         {
            // collision avoidance maneuvering. we can't step on our own feet given the 3 parameter model,
            // but we could avoid stepping on the opposite goal foot (not applicable when going to waypoints)
            ConvexPolygon2D candidatePolygon = createFootPolygon(candidate.get(side), 0.0);
            boolean collision = convexPolygonTools.doPolygonsIntersect(candidatePolygon, createFootPolygon(goal.get(side.getOppositeSide()), 0.04));
            // Theres only sometimes you care about this, it's really a thing to sometimes save 1 step when approaching a diagonal stance
            // TODO: Implement collision avoidance
            //   could be rotating away from the collision, rotating the foot, or bringing foot closer to hip (basically the 3 parameters, lol)

            // Calculate if direct step to goal is possible
            Vector3D goalForward = forward(goal.get(side));
            Pose3D goalHip = new Pose3D(0.0, side.negateIfRightSide(hipWidth), 0.0, 0.0, 0.0, 0.0);
            goalMid.transform(goalHip);

            // Limits are increased so as to not fail in the case of user-specified aggressive goal stances
            double goalHipToGoalFootYaw = yaw(forward(goalHip), goalForward);
            double goalFootYawMin = Math.min(goalHipToGoalFootYaw - Math.toRadians(1.0), footYawMin);
            double goalFootYawMax = Math.max(goalHipToGoalFootYaw + Math.toRadians(1.0), footYawMax);

            Vector3D goalHipLateral = new Vector3D(0.0, side.negateIfRightSide(1.0), 0.0);
            goalHip.transform(goalHipLateral);
            double goalHipToGoalStepAngle = yaw(goalHipLateral, sub(goal.get(side).getPosition(), goalHip.getPosition()));
            double goalStepAngleMin = Math.min(goalHipToGoalStepAngle - Math.toRadians(1.0), -stepAngleLimit);
            double goalStepAngleMax = Math.max(goalHipToGoalStepAngle + Math.toRadians(1.0), stepAngleLimit);

            double goalStepLengthMax = Math.max(goalHip.getPosition().distance(goal.get(side).getPosition()) + 0.01, stepLength);

            double requiredFootYaw = yaw(swingHipForward, goalForward);
            double requiredStepAngle = yaw(swingHipLateral, sub(goal.get(side).getPosition(), swingHip.get(side).getPosition()));
            double requiredLength = swingHip.get(side).getPosition().distance(goal.get(side).getPosition());

            boolean possible = requiredFootYaw >= goalFootYawMin && requiredFootYaw <= goalFootYawMax;
            possible &= requiredStepAngle >= goalStepAngleMin && requiredStepAngle <= goalStepAngleMax;
            possible &= requiredLength <= goalStepLengthMax;

            goalstepPossible.put(side, possible);
         }
      }

      if (waypoint.containsNaN())
         for (RobotSide side : RobotSide.values) // Take a step towards the goal
         {
            if (atGoal.get(side)) // Never step a foot already at goal
               continue;
            if (atGoal.get(side.getOppositeSide()) || goalstepPossible.get(side)) // Always take last goal step
            {
               footToSwing = side;
               swingEnd.set(goal.get(side));
               swingDistance = stance.get(side).getPosition().distance(goal.get(side).getPosition());
               swingDistance += stance.get(side).getOrientation().distance(goal.get(side).getOrientation()) * 0.1 / Math.toRadians(45.0);
               return false;
            }
         }

      // Step the candidate that's a bigger step
      footToSwing = RobotSide.LEFT;
      double distance = stance.get(footToSwing).getPosition().distance(candidate.get(footToSwing).getPosition());
      distance += stance.get(footToSwing).getOrientation().distance(candidate.get(footToSwing).getOrientation()) * 0.1 / Math.toRadians(45.0);
      double oppositeDistance = stance.get(footToSwing.getOppositeSide()).getPosition().distance(candidate.get(footToSwing.getOppositeSide()).getPosition());
      oppositeDistance += stance.get(footToSwing.getOppositeSide()).getOrientation()
                                .distance(candidate.get(footToSwing.getOppositeSide()).getOrientation()) * 0.1 / Math.toRadians(45.0);
      if (oppositeDistance > distance)
         footToSwing = footToSwing.getOppositeSide();
      swingDistance = Math.max(distance, oppositeDistance);
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

   private boolean isCrossover(Pose3D stance, Pose3D step, RobotSide stepSide, double more)
   {
      Vector3D stanceForward = forward(stance);
      Point3D checkPosition = new Point3D(0.0, stepSide.negateIfLeftSide(more), 0.0);
      stance.transform(checkPosition);
      Location location = EuclidGeometryTools.whichSideOfLine2DIsPoint2DOn(step.getPosition().getX(),
                                                                           step.getPosition().getY(),
                                                                           checkPosition.getX(),
                                                                           checkPosition.getY(),
                                                                           stanceForward.getX(),
                                                                           stanceForward.getY());
      return location == null || (stepSide == RobotSide.LEFT && location == Location.RIGHT
                               || stepSide == RobotSide.RIGHT && location == Location.LEFT);
   }

   // Compute sidewaysness: 1 straight sideways, 0.5 diagonal, 0 forward/backward
   // Vector2D stanceToGoal = new Vector2D();
   // stanceToGoal.sub(new Point2D(goalMid.getPosition()), new Point2D(stanceMid.getPosition()));
   // stanceToGoal.normalize();
   // Vector2D stanceMidForward = new Vector2D(Axis2D.X);
   // stanceMid.getOrientation().transform(stanceMidForward);
   // sidewaysness = 1.0 - (2.0 / Math.PI) * Math.abs(Math.asin(stanceMidForward.dot(stanceToGoal)));

   private Vector3D forward(Pose3D pose)
   {
      Vector3D forward = new Vector3D(Axis3D.X);
      pose.transform(forward);
      return forward;
   }

   private Vector3D forward(Quaternion quaternion)
   {
      Vector3D forward = new Vector3D(Axis3D.X);
      quaternion.transform(forward);
      return forward;
   }

   private Vector3D sub(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      Vector3D sub = new Vector3D();
      sub.sub(tuple1, tuple2);
      return sub;
   }

   private Vector3D direction(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      Vector3D direction = sub(tuple1, tuple2);
      direction.normalize();
      return direction;
   }

   private Vector3D cross(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      Vector3D cross = new Vector3D();
      cross.cross(tuple1, tuple2);
      return cross;
   }

   private double yaw(Vector3D from, Vector3D to)
   {
      return Math.signum(cross(from, to).getZ()) * TupleTools.angle(from, to);
   }

   public void setStepPlannedCallback(Runnable stepPlannedCallback)
   {
      this.stepPlannedCallback = stepPlannedCallback;
   }

   public void setMaxSteps(int maxSteps)
   {
      this.maxSteps = maxSteps;
   }

   public void setHipWidth(double hipWidth)
   {
      this.hipWidth = hipWidth;
   }

   public void setNextPelvisYawLimit(double nextPelvisYawLimit)
   {
      this.nextPelvisYawLimit = nextPelvisYawLimit;
   }

   public void setStepLength(double stepLength)
   {
      this.stepLength = stepLength;
   }

   public void setStepAngleLimit(double stepAngleLimit)
   {
      this.stepAngleLimit = stepAngleLimit;
   }

   public void setInwardLimit(double inwardLimit)
   {
      this.inwardLimit = inwardLimit;
   }

   public void setOutwardLimit(double outwardLimit)
   {
      this.outwardLimit = outwardLimit;
   }

   public RobotSide getFootToSwing()
   {
      return footToSwing;
   }

   public Pose3D getSwingEnd()
   {
      return swingEnd;
   }

   public Pose3D getGoalMid()
   {
      return goalMid;
   }

   public SideDependentList<Pose3D> getPelvis()
   {
      return pelvis;
   }

   public SideDependentList<Pose3D> getNextPelvis()
   {
      return nextPelvis;
   }

   public SideDependentList<Pose3D> getSwingHip()
   {
      return swingHip;
   }

   public SideDependentList<Vector3D> getToGoalLinear()
   {
      return toGoalLinear;
   }

   public Notification getPrintNotification()
   {
      return notification;
   }
}
