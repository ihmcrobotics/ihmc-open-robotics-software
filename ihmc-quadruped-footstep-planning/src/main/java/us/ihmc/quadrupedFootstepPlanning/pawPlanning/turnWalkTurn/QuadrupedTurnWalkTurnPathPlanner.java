package us.ihmc.quadrupedFootstepPlanning.pawPlanning.turnWalkTurn;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedPlanning.stepStream.bodyPath.QuadrupedBodyPathPlan;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedTurnWalkTurnPathPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private enum RobotSpeed
   {
      FAST, MEDIUM, SLOW
   }

   private static final Vector2DReadOnly forwardHeading = new Vector2D(1.0, 0.0);

   private final YoDouble maxYawRate = new YoDouble("maxYawRate", registry);

   private final YoDouble maxForwardAcceleration = new YoDouble("maxForwardAcceleration", registry);
   private final YoDouble maxYawAcceleration = new YoDouble("maxYawAcceleration", registry);

   private final YoDouble fastVelocity = new YoDouble("fastVelocity", registry);
   private final YoDouble mediumVelocity = new YoDouble("mediumVelocity", registry);
   private final YoDouble slowVelocity = new YoDouble("slowVelocity", registry);

   private final YoEnum<RobotSpeed> robotSpeed = YoEnum.create("robotSpeed", RobotSpeed.class, registry);

   private final QuadrupedBodyPathPlan pathPlan = new QuadrupedBodyPathPlan();
   private final BodyPathPlanner bodyPathPlanner;


   public QuadrupedTurnWalkTurnPathPlanner(TurnWalkTurnPathParameters pathParameters, BodyPathPlanner bodyPathPlanner, YoVariableRegistry parentRegistry)
   {
      this.bodyPathPlanner = bodyPathPlanner;

      robotSpeed.set(RobotSpeed.MEDIUM);

      maxYawRate.set(pathParameters.getMaxYawRate());

      maxForwardAcceleration.set(pathParameters.getMaxForwardAcceleration());
      maxYawAcceleration.set(pathParameters.getMaxYawAcceleration());

      fastVelocity.set(pathParameters.getFastVelocity());
      mediumVelocity.set(pathParameters.getMediumVelocity());
      slowVelocity.set(pathParameters.getSlowVelocity());

      parentRegistry.addChild(registry);
   }

   public void computePlan()
   {
      pathPlan.clear();

      BodyPathPlan bodyPathWaypoints = bodyPathPlanner.getPlan();
      Pose2DReadOnly startPose = bodyPathWaypoints.getStartPose();
      Pose2DReadOnly goalPose = bodyPathWaypoints.getGoalPose();

      pathPlan.setStartPose(startPose);
      pathPlan.setGoalPose(goalPose);

         computePlanDiscretelyTraversingWaypoints();

      pathPlan.setExpressedInAbsoluteTime(false);
   }

   public QuadrupedBodyPathPlan getPlan()
   {
      return pathPlan;
   }

   private static final Vector2DReadOnly zeroVelocity = new Vector2D();
   private final Vector2D desiredHeading = new Vector2D();


   private void computePlanDiscretelyTraversingWaypoints()
   {
      double currentTime = 0.0;
      BodyPathPlan bodyPathWaypoints = bodyPathPlanner.getPlan();
      Pose2DReadOnly startPose = bodyPathWaypoints.getStartPose();
      Point2D currentPosition = new Point2D(startPose.getPosition());
      double currentYaw = startPose.getYaw();
      Vector2D currentLinearVelocity = new Vector2D();

      for (int currentWaypointIndex = 0; currentWaypointIndex < bodyPathWaypoints.getNumberOfWaypoints() - 1; currentWaypointIndex++)
      {
         int nextWaypointIndex = currentWaypointIndex + 1;
         Point2DReadOnly nextWaypoint = new Point2D(bodyPathWaypoints.getWaypoint(nextWaypointIndex));

         desiredHeading.sub(nextWaypoint, currentPosition);
         desiredHeading.normalize();
         double desiredYaw = forwardHeading.angle(desiredHeading);

         double timeToAccelerateTurnWithNoMaxRate = QuadrupedTurnWalkTurnPathPlanner.computeTimeToAccelerateToAchieveValueWithNoMaxRate(currentYaw, 0.0, desiredYaw, 0.0, maxYawAcceleration.getDoubleValue());
         boolean reachesMaxYawRate = maxYawAcceleration.getDoubleValue() * timeToAccelerateTurnWithNoMaxRate > maxYawRate.getValue();

         // handle turning
         if (reachesMaxYawRate)
         {
            double angleDelta = desiredYaw - currentYaw;
            double errorSign = Math.signum(angleDelta);
            double timeToAccelerate = maxYawRate.getDoubleValue() / maxYawAcceleration.getValue();
            double deltaWhileAccelerating = 0.5 * errorSign * maxYawAcceleration.getValue() * MathTools.square(timeToAccelerate);

            currentYaw += deltaWhileAccelerating ;
            currentTime += timeToAccelerate;

            // add accelerating waypoint
            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, errorSign * maxYawRate.getDoubleValue(), currentTime);

            double timeForMaxVelocity = (Math.abs(angleDelta) - Math.abs(2.0 * deltaWhileAccelerating)) / maxYawRate.getDoubleValue();

            // add constant velocity waypoint
            currentYaw += timeForMaxVelocity * maxYawRate.getDoubleValue() * errorSign;
            currentTime += timeForMaxVelocity;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, errorSign * maxYawRate.getDoubleValue(), currentTime);

            currentYaw += deltaWhileAccelerating;
            currentTime += timeToAccelerate;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }
         else
         {
            double angleDelta = desiredYaw - currentYaw;
            double errorSign = Math.signum(angleDelta);

            double deltaWhileAccelerating = 0.5 * errorSign * maxYawAcceleration.getDoubleValue() * MathTools.square(timeToAccelerateTurnWithNoMaxRate);
            double velocityAfterAccelerating = errorSign * maxYawAcceleration.getDoubleValue() * timeToAccelerateTurnWithNoMaxRate;

            currentYaw += deltaWhileAccelerating ;
            currentTime += timeToAccelerateTurnWithNoMaxRate;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, velocityAfterAccelerating, currentTime);

            currentYaw += deltaWhileAccelerating;
            currentTime += timeToAccelerateTurnWithNoMaxRate;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }

         // handle translation
         double desiredSpeed;
         switch (robotSpeed.getEnumValue())
         {
         case FAST:
            desiredSpeed = fastVelocity.getDoubleValue();
            break;
         case MEDIUM:
            desiredSpeed = mediumVelocity.getDoubleValue();
            break;
         default:
            desiredSpeed = slowVelocity.getDoubleValue();
            break;
         }

         double distanceToTravel = currentPosition.distance(nextWaypoint);
         double timeToAccelerateDistanceWithNoMaxRate = QuadrupedTurnWalkTurnPathPlanner.computeTimeToAccelerateToAchieveValueWithNoMaxRate(0.0, 0.0, distanceToTravel, 0.0, maxForwardAcceleration.getDoubleValue());
         boolean reachesMaxForwardRate = maxForwardAcceleration.getDoubleValue() * timeToAccelerateDistanceWithNoMaxRate > desiredSpeed;
         if (reachesMaxForwardRate)
         {
            double timeToAccelerate = desiredSpeed / maxForwardAcceleration.getValue();
            double distanceWhileAccelerating = 0.5 * maxForwardAcceleration.getValue() * MathTools.square(timeToAccelerate);

            desiredHeading.normalize();
            desiredHeading.scale(distanceWhileAccelerating);

            currentPosition.add(desiredHeading);

            currentLinearVelocity.set(desiredHeading);
            currentLinearVelocity.normalize();
            currentLinearVelocity.scale(timeToAccelerate * maxForwardAcceleration.getDoubleValue());

            currentTime += timeToAccelerate;

            // add accelerating waypoint
            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), currentLinearVelocity, 0.0 , currentTime);

            double timeAtMaxVelocity = (Math.abs(distanceToTravel) - Math.abs(2.0 * distanceWhileAccelerating)) / desiredSpeed;

            // add constant velocity waypoint
            desiredHeading.set(currentLinearVelocity);
            desiredHeading.scale(timeAtMaxVelocity);

            currentPosition.add(desiredHeading);

            currentTime += timeAtMaxVelocity;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), currentLinearVelocity, 0.0, currentTime);

            desiredHeading.normalize();
            desiredHeading.scale(distanceWhileAccelerating);

            currentPosition.add(desiredHeading);

            currentTime += timeToAccelerate;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }
         else
         {
            double deltaWhileAccelerating = 0.5 * maxForwardAcceleration.getDoubleValue() * MathTools.square(timeToAccelerateDistanceWithNoMaxRate);
            double velocityAfterAccelerating = maxForwardAcceleration.getDoubleValue() * timeToAccelerateDistanceWithNoMaxRate;

            desiredHeading.normalize();
            desiredHeading.scale(deltaWhileAccelerating);

            currentPosition.add(desiredHeading);

            currentLinearVelocity.set(desiredHeading);
            currentLinearVelocity.normalize();
            currentLinearVelocity.scale(velocityAfterAccelerating);

            currentTime += timeToAccelerateDistanceWithNoMaxRate;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), currentLinearVelocity, 0.0, currentTime);

            currentPosition.add(desiredHeading);

            currentTime += timeToAccelerateDistanceWithNoMaxRate;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }
      }

      double goalYaw = bodyPathWaypoints.getGoalPose().getYaw();
      if (!MathTools.epsilonEquals(currentYaw, goalYaw, 2.0e-3))
      {
         double timeToAccelerateTurnWithNoMaxRate = QuadrupedTurnWalkTurnPathPlanner.computeTimeToAccelerateToAchieveValueWithNoMaxRate(currentYaw, 0.0, goalYaw, 0.0, maxYawAcceleration.getDoubleValue());
         boolean reachesMaxYawRate = maxYawAcceleration.getDoubleValue() * timeToAccelerateTurnWithNoMaxRate > maxYawRate.getValue();

         // handle turning
         if (reachesMaxYawRate)
         {
            double angleDelta = goalYaw - currentYaw;
            double errorSign = Math.signum(angleDelta);
            double timeToAccelerate = maxYawRate.getDoubleValue() / maxYawAcceleration.getValue();
            double deltaWhileAccelerating = 0.5 * errorSign * maxYawAcceleration.getValue() * MathTools.square(timeToAccelerate);

            currentYaw += deltaWhileAccelerating ;
            currentTime += timeToAccelerate;

            // add accelerating waypoint
            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, errorSign * maxYawRate.getDoubleValue(), currentTime);

            double timeForMaxVelocity = (Math.abs(angleDelta) - Math.abs(2.0 * deltaWhileAccelerating)) / maxYawRate.getDoubleValue();

            // add constant velocity waypoint
            currentYaw += timeForMaxVelocity * maxYawRate.getDoubleValue() * errorSign;
            currentTime += timeForMaxVelocity;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, errorSign * maxYawRate.getDoubleValue(), currentTime);

            currentYaw += deltaWhileAccelerating;
            currentTime += timeToAccelerate;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }
         else
         {
            double angleDelta = goalYaw - currentYaw;
            double errorSign = Math.signum(angleDelta);

            double deltaWhileAccelerating = 0.5 * errorSign * maxYawAcceleration.getDoubleValue() * MathTools.square(timeToAccelerateTurnWithNoMaxRate);
            double velocityAfterAccelerating = errorSign * maxYawAcceleration.getDoubleValue() * timeToAccelerateTurnWithNoMaxRate;

            currentYaw += deltaWhileAccelerating ;
            currentTime += timeToAccelerateTurnWithNoMaxRate;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, velocityAfterAccelerating, currentTime);

            currentYaw += deltaWhileAccelerating;
            currentTime += timeToAccelerateTurnWithNoMaxRate;

            pathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }
      }
   }


   static double computeTimeToAccelerateToAchieveValueWithNoMaxRate(double currentValue, double currentRate, double desiredValue, double desiredRate,
                                                                           double maxAcceleration)
   {
      double angleDelta = desiredValue - currentValue;
      double motionDirection = Math.signum(desiredValue - currentValue);
      double acceleration = motionDirection * maxAcceleration;
      double velocityDelta = 1.0 / (2.0 * acceleration) * (MathTools.square(currentRate) - MathTools.square(desiredRate));
      double a = acceleration;
      double b = 2.0 * currentRate;
      double c = -angleDelta + velocityDelta;

      return largestQuadraticSolution(a, b, c);
   }


   static double largestQuadraticSolution(double a, double b, double c)
   {
      double radical = Math.sqrt(MathTools.square(b) - 4.0 * a * c);

      if (a > 0)
      {
         return (-b + radical) / (2.0 * a);
      }
      else
      {
         return (-b - radical) / (2.0 * a);
      }
   }
}
