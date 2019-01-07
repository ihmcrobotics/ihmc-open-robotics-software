package us.ihmc.quadrupedPlanning.velocityPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedConstantAccelerationBodyPathPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private enum RobotSpeed
   {
      FAST, MEDIUM, SLOW
   }

   private static final Vector2DReadOnly forwardHeading = new Vector2D(1.0, 0.0);

   private final YoDouble maxForwardVelocity = new YoDouble("maxForwardVelocity", registry);
   private final YoDouble maxLateralVelocity = new YoDouble("maxLateralVelocity", registry);
   private final YoDouble maxYawRate = new YoDouble("maxYawRate", registry);

   private final YoDouble maxForwardAcceleration = new YoDouble("maxForwardAcceleration", registry);
   private final YoDouble maxLateralAcceleration = new YoDouble("maxLateralAcceleration", registry);
   private final YoDouble maxYawAcceleration = new YoDouble("maxYawAcceleration", registry);

   private final YoBoolean discretelyTraverseWaypoints = new YoBoolean("discretelyTraverseWaypoints", registry);

   private final YoDouble fastVelocity = new YoDouble("fastVelocity", registry);
   private final YoDouble mediumVelocity = new YoDouble("mediumVelocity", registry);
   private final YoDouble slowVelocity = new YoDouble("slowVelocity", registry);

   private final YoEnum<RobotSpeed> robotSpeed = YoEnum.create("robotSpeed", RobotSpeed.class, registry);

   private final QuadrupedBodyPathPlan bodyPathPlan = new QuadrupedBodyPathPlan();
   private BodyPathPlan bodyPathWaypoints;

   private final PoseReferenceFrame headingFrame = new PoseReferenceFrame("headingFrame", ReferenceFrame.getWorldFrame());

   public QuadrupedConstantAccelerationBodyPathPlanner(YoVariableRegistry parentRegistry)
   {
      robotSpeed.set(RobotSpeed.MEDIUM);
      discretelyTraverseWaypoints.set(true);

      parentRegistry.addChild(registry);
   }

   public void setBodyPathWaypoints(BodyPathPlan bodyPathWaypoints)
   {
      this.bodyPathWaypoints = bodyPathWaypoints;
   }

   public void computePlan()
   {
      bodyPathPlan.clear();

      Pose2DReadOnly startPose = bodyPathWaypoints.getStartPose();
      Pose2DReadOnly goalPose = bodyPathWaypoints.getGoalPose();

      bodyPathPlan.setStartPose(startPose);
      bodyPathPlan.setGoalPose(goalPose);

      if (discretelyTraverseWaypoints.getBooleanValue())
      {
         computePlanDiscretelyTraversingWaypoints();
      }
   }

   public QuadrupedBodyPathPlan getPlan()
   {
      return bodyPathPlan;
   }

   private static final Vector2DReadOnly zeroVelocity = new Vector2D();
   private final Vector2D desiredHeading = new Vector2D();
   private final Vector2D previousHeading = new Vector2D();


   private void computePlanDiscretelyTraversingWaypoints()
   {
      double currentTime = 0.0;
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
         double desiredYaw = desiredHeading.angle(forwardHeading);

         double timeToAccelerateTurnWithNoMaxRate = QuadrupedBodyPathTools.computeTimeToAccelerateToAchieveValueWithNoMaxRate(currentYaw, 0.0, desiredYaw, 0.0, maxYawAcceleration.getDoubleValue());
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
            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, errorSign * maxYawRate.getDoubleValue(), currentTime);

            double timeForMaxVelocity = (Math.abs(angleDelta) - Math.abs(2.0 * deltaWhileAccelerating)) / maxYawRate.getDoubleValue();

            // add constant velocity waypoint
            currentYaw += timeForMaxVelocity * maxYawRate.getDoubleValue() * errorSign;
            currentTime += timeForMaxVelocity;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, errorSign * maxYawRate.getDoubleValue(), currentTime);

            currentYaw += deltaWhileAccelerating;
            currentTime += timeToAccelerate;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }
         else
         {
            double angleDelta = desiredYaw - currentYaw;
            double errorSign = Math.signum(angleDelta);

            double deltaWhileAccelerating = 0.5 * errorSign * maxYawAcceleration.getDoubleValue() * MathTools.square(timeToAccelerateTurnWithNoMaxRate);
            double velocityAfterAccelerating = errorSign * maxYawAcceleration.getDoubleValue() * timeToAccelerateTurnWithNoMaxRate;

            currentYaw += deltaWhileAccelerating ;
            currentTime += timeToAccelerateTurnWithNoMaxRate;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, velocityAfterAccelerating, currentTime);

            currentYaw += deltaWhileAccelerating;
            currentTime += timeToAccelerateTurnWithNoMaxRate;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
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
         double timeToAccelerateDistanceWithNoMaxRate = QuadrupedBodyPathTools.computeTimeToAccelerateToAchieveValueWithNoMaxRate(0.0, 0.0, distanceToTravel, 0.0, maxForwardAcceleration.getDoubleValue());
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
            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), currentLinearVelocity, 0.0 , currentTime);

            double timeAtMaxVelocity = (Math.abs(distanceToTravel) - Math.abs(2.0 * distanceWhileAccelerating)) / maxForwardVelocity.getDoubleValue();

            // add constant velocity waypoint
            desiredHeading.set(currentLinearVelocity);
            desiredHeading.scale(timeAtMaxVelocity);

            currentPosition.add(desiredHeading);

            currentTime += timeAtMaxVelocity;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), currentLinearVelocity, 0.0, currentTime);

            desiredHeading.normalize();
            desiredHeading.scale(distanceWhileAccelerating);

            currentPosition.add(desiredHeading);

            currentTime += timeToAccelerate;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
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

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), currentLinearVelocity, 0.0, currentTime);

            currentPosition.add(desiredHeading);

            currentTime += timeToAccelerateDistanceWithNoMaxRate;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }
      }
   }
   /*

   private void computePlanSmoothlyTraversingWaypoints()
   {
      double currentTime = 0.0;
      Pose2DReadOnly startPose = bodyPathWaypoints.getStartPose();
      Point2D currentPosition = new Point2D(startPose.getPosition());
      double currentYaw = startPose.getYaw();
      Vector2D currentLinearVelocity = new Vector2D();

      previousHeading.set(Math.cos(currentYaw), Math.sin(currentYaw));

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


      for (int currentWaypointIndex = 0; currentWaypointIndex < bodyPathWaypoints.getNumberOfWaypoints() - 1; currentWaypointIndex++)
      {
         int nextWaypointIndex = currentWaypointIndex + 1;
         Point2DReadOnly nextWaypoint = new Point2D(bodyPathWaypoints.getWaypoint(nextWaypointIndex));

         desiredHeading.sub(nextWaypoint, currentPosition);
         desiredHeading.normalize();

         double desiredYaw;
         if (currentWaypointIndex == 0)
         {
            desiredYaw = desiredHeading.angle(forwardHeading);
         }
         else
         {
            double currentDesiredYaw = desiredHeading.angle(forwardHeading);
//            Vector2D previousHeading =
         }


         double timeToAccelerateTurnWithNoMaxRate = QuadrupedBodyPathTools.computeTimeToAccelerateToAchieveValueWithNoMaxRate(currentYaw, 0.0, desiredYaw, 0.0, maxYawAcceleration.getDoubleValue());
         boolean reachesMaxYawRate = maxYawAcceleration.getDoubleValue() * timeToAccelerateTurnWithNoMaxRate > maxYawRate.getValue();

         if (reachesMaxYawRate)
         {
            double angleDelta = desiredYaw - currentYaw;
            double errorSign = Math.signum(angleDelta);
            double timeToAccelerate = maxYawRate.getDoubleValue() / maxYawAcceleration.getValue();
            double deltaWhileAccelerating = 0.5 * errorSign * maxYawAcceleration.getValue() * MathTools.square(timeToAccelerate);

            double yawAfterTurning = currentYaw + deltaWhileAccelerating;

            Vector2D desiredVelocityAfterAcceleratingYaw = new Vector2D();
            Vector2D distanceWhileAcceleratingYaw = new Vector2D();

            computeDesiredVelocityWithAccelerationLimits(desiredVelocityAfterAcceleratingYaw, currentLinearVelocity, desiredHeading, yawAfterTurning, desiredSpeed,
                                                         timeToAccelerate);

            distanceWhileAcceleratingYaw.add(desiredVelocityAfterAcceleratingYaw, currentLinearVelocity);
            distanceWhileAcceleratingYaw.scale(0.5 * timeToAccelerate);


            currentYaw += deltaWhileAccelerating ;
            currentTime += timeToAccelerate;

            // add accelerating waypoint
            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, errorSign * maxYawRate.getDoubleValue(), currentTime);

            double timeForMaxVelocity = (Math.abs(angleDelta) - Math.abs(2.0 * deltaWhileAccelerating)) / maxYawRate.getDoubleValue();

            // add constant velocity waypoint
            currentYaw += timeForMaxVelocity * maxYawRate.getDoubleValue() * errorSign;
            currentTime += timeForMaxVelocity;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, errorSign * maxYawRate.getDoubleValue(), currentTime);

            currentYaw += deltaWhileAccelerating;
            currentTime += timeToAccelerate;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }
         else
         {
            double angleDelta = desiredYaw - currentYaw;
            double errorSign = Math.signum(angleDelta);

            double deltaWhileAccelerating = 0.5 * errorSign * maxYawAcceleration.getDoubleValue() * MathTools.square(timeToAccelerateTurnWithNoMaxRate);
            double velocityAfterAccelerating = errorSign * maxYawAcceleration.getDoubleValue() * timeToAccelerateTurnWithNoMaxRate;

            currentYaw += deltaWhileAccelerating ;
            currentTime += timeToAccelerateTurnWithNoMaxRate;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, velocityAfterAccelerating, currentTime);

            currentYaw += deltaWhileAccelerating;
            currentTime += timeToAccelerateTurnWithNoMaxRate;

            bodyPathPlan.addWaypoint(new Pose2D(currentPosition, currentYaw), zeroVelocity, 0.0, currentTime);
         }

         previousHeading.set(desiredHeading);
      }
   }
   */

   private final FrameVector2D maxAccelerationInWorld = new FrameVector2D();
   private final FrameVector2D maxVelocityChange = new FrameVector2D();
   private final Quaternion orientation = new Quaternion();

   private void computeDesiredVelocityWithAccelerationLimits(Vector2D desiredVelocityToPack, Vector2DReadOnly currentVelocity, Vector2DReadOnly desiredHeading,
                                                             double currentYaw, double desiredSpeed, double timeToAcceleration)
   {
      desiredVelocityToPack.set(desiredHeading);
      desiredVelocityToPack.scale(desiredSpeed / desiredHeading.length());

      orientation.setToYawQuaternion(currentYaw);
      headingFrame.setOrientationAndUpdate(orientation);

      maxAccelerationInWorld.setIncludingFrame(headingFrame, maxForwardAcceleration.getDoubleValue(), maxLateralAcceleration.getDoubleValue());
      maxAccelerationInWorld.changeFrame(ReferenceFrame.getWorldFrame());

      maxVelocityChange.setIncludingFrame(maxAccelerationInWorld);
      maxVelocityChange.scale(timeToAcceleration);

      if (Math.abs(desiredVelocityToPack.getX() - currentVelocity.getX()) > maxVelocityChange.getX())
      {
         double alpha = Math.abs(maxVelocityChange.getX() / desiredVelocityToPack.getX());
         desiredVelocityToPack.scale(alpha);
      }
      if (Math.abs(desiredVelocityToPack.getY() - currentVelocity.getY()) > maxVelocityChange.getY())
      {
         double alpha = Math.abs(maxVelocityChange.getY() / desiredVelocityToPack.getY());
         desiredVelocityToPack.scale(alpha);
      }
   }
}
