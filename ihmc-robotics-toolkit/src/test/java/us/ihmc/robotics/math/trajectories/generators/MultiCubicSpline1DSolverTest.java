package us.ihmc.robotics.math.trajectories.generators;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.robotics.Assert;

public class MultiCubicSpline1DSolverTest
{
   private final double permissableError = 1e-3;
   @Test
   public void testNotUsingNativeCommonOps() {
      DMatrixRMaj mat = new DMatrixRMaj(1, 1);
      MultiCubicSpline1DSolver solverUsingNative = new MultiCubicSpline1DSolver(true);
      solverUsingNative.setEndpoints(0, 1, 0, 1);
      solverUsingNative.addWaypoint(2, 0.25);
      solverUsingNative.addWaypoint(-2, 0.75);
      double costUsingNative = solverUsingNative.solve(mat);

      mat = new DMatrixRMaj(1, 1);
      MultiCubicSpline1DSolver solverNotUsingNative = new MultiCubicSpline1DSolver(false);
      solverNotUsingNative.setEndpoints(0, 1, 0, 1);
      solverNotUsingNative.addWaypoint(2, 0.25);
      solverNotUsingNative.addWaypoint(-2, 0.75);
      double costNotUsingNative = solverNotUsingNative.solve(mat);

      double percentDiff = 100 * Math.abs(costUsingNative - costNotUsingNative) / costNotUsingNative;
      Assert.assertTrue(percentDiff < permissableError);
   }
}
