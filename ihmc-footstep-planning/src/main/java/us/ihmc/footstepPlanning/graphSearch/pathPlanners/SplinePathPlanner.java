package us.ihmc.footstepPlanning.graphSearch.pathPlanners;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SplinePathPlanner extends AbstractWaypointsForFootstepsPlanner
{
   private static final int numberOfPoints = 5;

   private final YoPolynomial xPoly;
   private final YoPolynomial yPoly;
   private final YoPolynomial zPoly;
   private final YoPolynomial yawPoly;

   public SplinePathPlanner(FootstepPlannerParametersReadOnly parameters, YoVariableRegistry parentRegistry)
   {
      super(parameters, parentRegistry);

      xPoly = new YoPolynomial("xPoly", 4, parentRegistry);
      yPoly = new YoPolynomial("yPoly", 4, parentRegistry);
      zPoly = new YoPolynomial("zPoly", 4, parentRegistry);
      yawPoly = new YoPolynomial("yawPoly", 4, parentRegistry);
   }

   public FootstepPlanningResult planWaypoints()
   {
      waypoints.clear();
      double yaw = bodyStartPose.getYaw();
      xPoly.setQuadratic(0.0, 1.0, bodyStartPose.getX(), Math.cos(yaw) * 0.2, bodyGoalPose.getX());
      yPoly.setQuadratic(0.0, 1.0, bodyStartPose.getY(), Math.sin(yaw) * 0.2, bodyGoalPose.getY());
      zPoly.setQuadratic(0.0, 1.0, bodyStartPose.getZ(), 0.0, bodyGoalPose.getZ());
      yawPoly.setQuadratic(0.0, 1.0, bodyStartPose.getYaw(), 0.0, bodyGoalPose.getYaw());
      for (int i = 0; i < numberOfPoints; i++)
      {
         double percent = i / (double) (numberOfPoints - 1);
         xPoly.compute(percent);
         yPoly.compute(percent);
         zPoly.compute(percent);
         yawPoly.compute(percent);
         Pose3D pose = new Pose3D();
         pose.setPosition(xPoly.getPosition(), yPoly.getPosition(), zPoly.getPosition());
         pose.setOrientationYawPitchRoll(yawPoly.getPosition(), 0.0, 0.0);
         waypoints.add(pose);
      }

      yoResult.set(FootstepPlanningResult.OPTIMAL_SOLUTION);
      return yoResult.getEnumValue();
   }

   public PlannerStatistics<?> getPlannerStatistics()
   {
      return null;
   }

   public void cancelPlanning(){}

   public void setTimeout(double timeout){}
}
