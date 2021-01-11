package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import static us.ihmc.robotics.Assert.assertEquals;

public class PolynomialEstimatorTest
{
   private static final double epsilon = 1e-4;

   @Test
   public void testLinear()
   {
      PolynomialEstimator estimator = new PolynomialEstimator();
      estimator.reshape(2);

      estimator.addObjectivePosition(0.0, 2.0);
      estimator.addObjectivePosition(2.0, 4.0);

      estimator.solve();

      assertEquals(2, estimator.getOrder());

      DMatrixRMaj coefficients = estimator.getCoefficients();
      assertEquals(2, coefficients.getNumRows());
      assertEquals(1, coefficients.getNumCols());

      assertEquals(2.0, coefficients.get(0, 0), epsilon);
      assertEquals(1.0, coefficients.get(1, 0), epsilon);

      estimator.compute(0.0);
      assertEquals(2.0, estimator.getPosition(), epsilon);
      assertEquals(1.0, estimator.getVelocity(), epsilon);
      assertEquals(0.0, estimator.getAcceleration(), epsilon);

      estimator.compute(2.0);
      assertEquals(4.0, estimator.getPosition(), epsilon);
      assertEquals(1.0, estimator.getVelocity(), epsilon);
      assertEquals(0.0, estimator.getAcceleration(), epsilon);
   }
}
