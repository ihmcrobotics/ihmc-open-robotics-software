package us.ihmc.quadrupedFootstepPlanning.pathPlanning;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SplinePawPathPlanner extends AbstractWaypointsForPawStepPlanner
{
   private static final int numberOfPoints = 5;

   private final YoPolynomial xPoly;
   private final YoPolynomial yPoly;
   private final YoPolynomial zPoly;

   public SplinePawPathPlanner(YoVariableRegistry registry)
   {
      this("", registry);
   }

   public SplinePawPathPlanner(String prefix, YoVariableRegistry registry)
   {
      super(prefix, registry);

      xPoly = new YoPolynomial("xPoly", 4, registry);
      yPoly = new YoPolynomial("yPoly", 4, registry);
      zPoly = new YoPolynomial("zPoly", 4, registry);
   }

   public PawStepPlanningResult planWaypoints()
   {
      waypoints.clear();
      double yaw = bodyStartPose.getYaw();
      xPoly.setQuadratic(0.0, 1.0, bodyStartPose.getX(), Math.cos(yaw) * 0.2, bodyGoalPose.getX());
      yPoly.setQuadratic(0.0, 1.0, bodyStartPose.getY(), Math.sin(yaw) * 0.2, bodyGoalPose.getY());
      zPoly.setQuadratic(0.0, 1.0, bodyStartPose.getZ(), Math.sin(yaw) * 0.2, bodyGoalPose.getZ());
      for (int i = 0; i < numberOfPoints; i++)
      {
         double percent = i / (double) (numberOfPoints - 1);
         xPoly.compute(percent);
         yPoly.compute(percent);
         zPoly.compute(percent);
         Point3D point = new Point3D(xPoly.getPosition(), yPoly.getPosition(), zPoly.getPosition());
         waypoints.add(point);
      }

      yoResult.set(PawStepPlanningResult.OPTIMAL_SOLUTION);
      return yoResult.getEnumValue();
   }
}
