package us.ihmc.quadrupedPlanning.pathPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedConstantAccelerationBodyPathPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final Vector2DReadOnly forwardHeading = new Vector2D(1.0, 0.0);

   private final YoDouble maxForwardVelocity = new YoDouble("maxForwardVelocity", registry);
   private final YoDouble maxLateralVelocity = new YoDouble("maxLateralVelocity", registry);
   private final YoDouble maxYawRate = new YoDouble("maxYawRate", registry);

   private final YoDouble maxForwardAcceleration = new YoDouble("maxForwardAcceleration", registry);
   private final YoDouble maxLateralAcceleration = new YoDouble("maxLateralAcceleration", registry);
   private final YoDouble maxYawAcceleration = new YoDouble("maxYawAcceleration", registry);

   private final QuadrupedBodyPathPlan bodyPathPlan = new QuadrupedBodyPathPlan();
   private BodyPathPlan bodyPathWaypoints;

   public QuadrupedConstantAccelerationBodyPathPlanner()
   {}

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

      double currentTime = 0.0;
      Point2D currentPosition = new Point2D(startPose.getPosition());
      double currentYaw = startPose.getYaw();
      Vector2D currentLinearVelocity = new Vector2D();
      double currentYawRate = 0.0;

      Vector2D currentHeading = new Vector2D();

      Vector2D desiredHeading = new Vector2D();

      for (int currentWaypointIndex = 0; currentWaypointIndex < bodyPathWaypoints.getNumberOfWaypoints() - 1; currentWaypointIndex++)
      {
         int nextWaypointIndex = currentWaypointIndex + 1;
         Point2DReadOnly currentWaypoint = new Point2D(bodyPathWaypoints.getWaypoint(currentWaypointIndex));
         Point2DReadOnly nextWaypoint = new Point2D(bodyPathWaypoints.getWaypoint(nextWaypointIndex));

         desiredHeading.sub(nextWaypoint, currentWaypoint);
         desiredHeading.normalize();
         double desiredYaw = desiredHeading.angle(forwardHeading);


      }
   }

   private static double getTurnTimeToDesiredHeading(double currentYaw, double currentYawRate, double desiredYaw)
   {
      double yawError = desiredYaw - currentYaw;
      double errorSign = Math.signum(yawError);

      return 12.0;
   }

}
