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
   private final SideDependentList<Pose3D> stance = new SideDependentList<>(() -> new Pose3D());
   private final SideDependentList<Pose3D> goal = new SideDependentList<>(() -> new Pose3D());
   private final Pose3D swingEnd = new Pose3D();
   private RobotSide footToSwing = RobotSide.LEFT;
   private final SideDependentList<Line3D> hipLine = new SideDependentList<>(() -> new Line3D());
   private final Pose3D stanceMid = new Pose3D();
   private final Pose3D goalMid = new Pose3D();
   private final SideDependentList<Point3D> swingHip = new SideDependentList<>(() -> new Point3D());
   private final SideDependentList<Point3D> goalHip = new SideDependentList<>(() -> new Point3D());
   private Runnable stepPlannedCallback = () -> {};
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   // --- Body path support (optional) ---
   private boolean useBodyPath = false;
   private final List<Pose3D> bodyPathWaypoints = new ArrayList<>();
   private final List<Point3D> bodyPathSamples = new ArrayList<>();
   private final List<Double> bodyPathS = new ArrayList<>();
   private double bodyPathLength = 0.0;
   private double currentS = 0.0;
   private double goalS = 0.0;
   private final double bodyPathStepLength = 0.20;
   // Alignment before stepping fully along body path
   private boolean aligningToBodyPath = false;
   private final double bodyPathAlignmentYawThreshold = Math.toRadians(10.0); // stop aligning when within 10 deg
   private RobotSide lastFootSwung = RobotSide.RIGHT; // so first step is LEFT by default

   public List<Pair<RobotSide, Pose3D>> plan(EnumMap<RobotSide, Pose3D> stance, EnumMap<RobotSide, Pose3D> goal)
   {
      for (RobotSide side : RobotSide.values)
      {
         this.goal.get(side).set(goal.get(side));
         this.stance.get(side).set(stance.get(side));
      }
      lastFootSwung = RobotSide.RIGHT;
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

   // Configure an optional body path. If waypoints is null or size < 2, body path is disabled.
   public void setBodyPath(List<Pose3D> waypoints)
   {
      useBodyPath = false;
      bodyPathWaypoints.clear();
      bodyPathSamples.clear();
      bodyPathS.clear();
      bodyPathLength = 0.0;
      currentS = 0.0;
      goalS = 0.0;

      if (waypoints == null || waypoints.size() < 2)
         return;

      bodyPathWaypoints.addAll(waypoints);
      resampleBodyPath();
      useBodyPath = !bodyPathSamples.isEmpty();
   }

   // Convenience overload: plan using optional body path.
   public List<Pair<RobotSide, Pose3D>> plan(EnumMap<RobotSide, Pose3D> stance,
                                             EnumMap<RobotSide, Pose3D> goal,
                                             List<Pose3D> bodyPath)
   {
      setBodyPath(bodyPath);
      return plan(stance, goal); // calls the existing plan()
   }


   private boolean planStep()
   {
      SideDependentList<Boolean> atGoal = new SideDependentList<>(side -> stance.get(side).getPosition().distance(goal.get(side).getPosition()) <= 0.01
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

      SideDependentList<Pose3D> candidate = new SideDependentList<>(() -> new Pose3D());
      SideDependentList<Boolean> goalstepPossible = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         candidate.get(side).set(stance.get(side));
         candidate.get(side).getOrientation().set(swingEndOrientation);

         if (useBodyPath)
         {
            if (side == RobotSide.LEFT)
            {
               // Initialize once per planStep
               initializeBodyPathParameters(stanceMid, goalMid, bodyPathStepLength);
            }

            double pathYaw   = computePathYawAtS(currentS);
            double stanceYaw = stanceMid.getYaw();
            double yawError  = normalizeAngle(pathYaw - stanceYaw);

            if (aligningToBodyPath && Math.abs(yawError) <= bodyPathAlignmentYawThreshold)
            {
               // Alignment complete, switch to normal path stepping
               aligningToBodyPath = false;
            }

            if (aligningToBodyPath)
            {
               // For each side, propose a small turning step around its own current stance.
               // Which foot actually moves is still decided later by your existing "bigger step" logic.
               Pose3D currentStance = stance.get(side);
               Pose3D cand          = candidate.get(side);

               cand.set(currentStance);

               // Small forward + yaw increment toward pathYaw, around this foot
               double maxTurnPerStep = stepYaw; // reuse your existing yaw bound
               double turn = Math.max(-maxTurnPerStep,
                                      Math.min(maxTurnPerStep, yawError));

               // Slight forward movement to avoid exact in‑place steps
               double smallForward = 0.02;
               Vector3D forward = new Vector3D(1.0, 0.0, 0.0);
               currentStance.getOrientation().transform(forward);
               forward.scale(smallForward);
               cand.getPosition().add(forward);

               double newYaw = currentStance.getYaw() + turn;
               cand.getOrientation().setToYawOrientation(newYaw);
            }
            else
            {
               // Normal body‑path step: mid‑pose along path + lateral offset
               Pose3D midOnPath = computeNextMidPoseOnPath(bodyPathStepLength);
               buildCandidateFeetFromMidPose(midOnPath, candidate);
            }
         }
         else
         {
            outer: // Calculate some possible steps toward the goal (original behavior)
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
                        AxisAngle axisAngle = new AxisAngle(Axis3D.Z,
                                                            Math.PI / 2.0 / resolution * side.negateIfRightSide(direction));
                        axisAngle.transform(ray);
                        candidate.get(side).getPosition().add(swingHip.get(side), ray);
                     }
                     else
                        break outer;
                  }
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
      if (useBodyPath && aligningToBodyPath)
      {
         // During alignment, force strict alternation
         footToSwing = lastFootSwung.getOppositeSide();
      }
      else
      {
         // Original rule: step the candidate that's a bigger step
         footToSwing = RobotSide.LEFT;
         double distance = stance.get(footToSwing).getPosition().distance(candidate.get(footToSwing).getPosition());
         double oppositeDistance = stance.get(footToSwing.getOppositeSide()).getPosition().distance(candidate.get(footToSwing.getOppositeSide()).getPosition());
         if (oppositeDistance > distance)
            footToSwing = footToSwing.getOppositeSide();
      }

      swingEnd.set(candidate.get(footToSwing));
      lastFootSwung = footToSwing; // remember for next step
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

   // --- Body path helpers (keep simple) ---

   private void resampleBodyPath()
   {
      bodyPathSamples.clear();
      bodyPathS.clear();
      bodyPathLength = 0.0;

      if (bodyPathWaypoints.size() < 2)
         return;

      Pose3D prev = bodyPathWaypoints.get(0);
      bodyPathSamples.add(new Point3D(prev.getPosition()));
      bodyPathS.add(0.0);

      double ds = 0.02; // 2 cm resolution
      for (int i = 1; i < bodyPathWaypoints.size(); i++)
      {
         Pose3D next = bodyPathWaypoints.get(i);
         double segmentLength = prev.getTranslation().distance(next.getTranslation());
         int numSteps = Math.max(1, (int) Math.round(segmentLength / ds));

         for (int k = 1; k <= numSteps; k++)
         {
            double alpha = (double) k / (double) numSteps;
            Point3D p = new Point3D();
            p.interpolate(prev.getPosition(), next.getPosition(), alpha);
            bodyPathLength += p.distance(bodyPathSamples.get(bodyPathSamples.size() - 1));
            bodyPathSamples.add(p);
            bodyPathS.add(bodyPathLength);
         }

         prev = next;
      }
   }

   private void initializeBodyPathParameters(Pose3D stanceMid, Pose3D goalMid, double stepLength)
   {
      if (bodyPathSamples.isEmpty())
      {
         useBodyPath = false;
         aligningToBodyPath = false;
         return;
      }

      currentS = projectPointToPathS(new Point3D(stanceMid.getPosition()));
      goalS    = projectPointToPathS(new Point3D(goalMid.getPosition()));
      if (Math.abs(goalS - currentS) < 1e-3)
         goalS = currentS + stepLength;

      // Start alignment phase if yaw differs significantly from tangent
      double pathYaw = computePathYawAtS(currentS);
      double stanceYaw = stanceMid.getYaw();
      double yawError = normalizeAngle(pathYaw - stanceYaw);
      aligningToBodyPath = Math.abs(yawError) > bodyPathAlignmentYawThreshold;
   }

   private double computePathYawAtS(double s)
   {
      if (bodyPathSamples.isEmpty())
         return stanceMid.getYaw();

      double epsilon = 0.01;
      double sPrev = Math.max(0.0, s - epsilon);
      double sNext = Math.min(bodyPathLength, s + epsilon);
      Point3D pPrev = interpolatePathAtS(sPrev);
      Point3D pNext = interpolatePathAtS(sNext);
      double dx = pNext.getX() - pPrev.getX();
      double dy = pNext.getY() - pPrev.getY();
      return Math.atan2(dy, dx);
   }

   private double translationDistance(Pose3D from, Pose3D to)
   {
      return from.getPosition().distance(to.getPosition());
   }

   private double normalizeAngle(double angle)
   {
      while (angle > Math.PI)
         angle -= 2.0 * Math.PI;
      while (angle < -Math.PI)
         angle += 2.0 * Math.PI;
      return angle;
   }

   private Pose3D computeNextMidPoseOnPath(double stepLength)
   {
      Pose3D midPose = new Pose3D();
      if (bodyPathSamples.isEmpty())
      {
         midPose.set(stanceMid);
         return midPose;
      }

      double direction = Math.signum(goalS - currentS);
      if (direction == 0.0)
         direction = 1.0;

      double targetS = currentS + direction * stepLength;
      if (direction > 0.0)
         targetS = Math.min(targetS, goalS);
      else
         targetS = Math.max(targetS, goalS);

      Point3D pos = interpolatePathAtS(targetS);
      currentS = targetS;

      midPose.getPosition().set(pos);

      double epsilon = 0.01;
      double sPrev = Math.max(0.0, targetS - epsilon);
      double sNext = Math.min(bodyPathLength, targetS + epsilon);
      Point3D pPrev = interpolatePathAtS(sPrev);
      Point3D pNext = interpolatePathAtS(sNext);
      double dx = pNext.getX() - pPrev.getX();
      double dy = pNext.getY() - pPrev.getY();
      double yaw = Math.atan2(dy, dx);
      midPose.getOrientation().setToYawOrientation(yaw);

      return midPose;
   }

   private double projectPointToPathS(Point3D point)
   {
      double bestS = 0.0;
      double bestDist = Double.POSITIVE_INFINITY;

      for (int i = 0; i < bodyPathSamples.size(); i++)
      {
         double dist = bodyPathSamples.get(i).distance(point);
         if (dist < bestDist)
         {
            bestDist = dist;
            bestS = bodyPathS.get(i);
         }
      }

      return bestS;
   }

   private Point3D interpolatePathAtS(double s)
   {
      if (s <= 0.0)
         return new Point3D(bodyPathSamples.get(0));
      if (s >= bodyPathLength)
         return new Point3D(bodyPathSamples.get(bodyPathSamples.size() - 1));

      for (int i = 1; i < bodyPathS.size(); i++)
      {
         double s0 = bodyPathS.get(i - 1);
         double s1 = bodyPathS.get(i);
         if (s >= s0 && s <= s1)
         {
            double alpha = (s - s0) / (s1 - s0);
            Point3D p = new Point3D();
            p.interpolate(bodyPathSamples.get(i - 1), bodyPathSamples.get(i), alpha);
            return p;
         }
      }

      return new Point3D(bodyPathSamples.get(bodyPathSamples.size() - 1));
   }

   private void buildCandidateFeetFromMidPose(Pose3D midPose, SideDependentList<Pose3D> candidate)
   {
      double hipWidth = 0.12;
      Vector3D lateral = new Vector3D(0.0, 1.0, 0.0);
      midPose.getOrientation().transform(lateral);

      for (RobotSide side : RobotSide.values)
      {
         double sign = side.negateIfRightSide(1.0);
         Vector3D offset = new Vector3D(lateral);
         offset.scale(sign * hipWidth);

         candidate.get(side).getPosition().set(midPose.getPosition());
         candidate.get(side).getPosition().add(offset);
         candidate.get(side).getOrientation().set(midPose.getOrientation());
      }
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

   public SideDependentList<Point3D> getSwingHip()
   {
      return swingHip;
   }

   public SideDependentList<Point3D> getGoalHip()
   {
      return goalHip;
   }

   public SideDependentList<Line3D> getHipLine()
   {
      return hipLine;
   }

   public Point3D getOppositeStance()
   {
      return stance.get(footToSwing.getOppositeSide()).getPosition();
   }
}
