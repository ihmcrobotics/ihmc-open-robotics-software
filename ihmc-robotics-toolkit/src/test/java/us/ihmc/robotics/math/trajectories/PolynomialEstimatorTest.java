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

      // do the same test, but with a higher weight
      estimator.reset();

      estimator.addObjectivePosition(10.0, 0.0, 2.0);
      estimator.addObjectivePosition(10.0, 2.0, 4.0);

      estimator.solve();

      assertEquals(2, estimator.getOrder());

      coefficients = estimator.getCoefficients();
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

      // do the same test, but with a velocity guess
      estimator.reset();

      estimator.addObjectivePosition(0.0, 2.0);
      estimator.addObjectiveVelocity(0.0, 1.0);

      estimator.solve();

      assertEquals(2, estimator.getOrder());

      coefficients = estimator.getCoefficients();
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

   @Test
   public void testCubic()
   {
      PolynomialEstimator estimator = new PolynomialEstimator();
      estimator.reshape(4);

      double duration = 2.0;
      double x0 = 2.0;
      double xf = 4.0;
      double v0 = 2.0;
      double vf = -2.0;
      estimator.addObjectivePosition(0.0, x0);
      estimator.addObjectiveVelocity(0.0, v0);
      estimator.addObjectivePosition(duration, xf);
      estimator.addObjectiveVelocity(duration, vf);

      estimator.solve();

      assertEquals(4, estimator.getOrder());

      DMatrixRMaj coefficients = estimator.getCoefficients();
      assertEquals(4, coefficients.getNumRows());
      assertEquals(1, coefficients.getNumCols());

      double c0 = x0;
      double c1 = v0;
      double c2 = 3.0 / (duration * duration) * (xf - x0) - 1.0 / duration * (2.0 * v0 + vf);
      double c3 = 2.0 / (duration * duration * duration) * (x0 - xf) + 1.0 / (duration * duration) * (v0 + vf);
      assertEquals(c0, coefficients.get(0, 0), epsilon);
      assertEquals(c1, coefficients.get(1, 0), epsilon);
      assertEquals(c2, coefficients.get(2, 0), epsilon);
      assertEquals(c3, coefficients.get(3, 0), epsilon);

      double a0 = 2.0 * c2;
      double af = a0 + 6.0 * duration * c3;


      estimator.compute(0.0);
      assertEquals(x0, estimator.getPosition(), epsilon);
      assertEquals(v0, estimator.getVelocity(), epsilon);
      assertEquals(a0, estimator.getAcceleration(), epsilon);

      estimator.compute(2.0);
      assertEquals(xf, estimator.getPosition(), epsilon);
      assertEquals(vf, estimator.getVelocity(), epsilon);
      assertEquals(af, estimator.getAcceleration(), epsilon);

      // do the same test, but with a higher weight
      estimator.reset();

      estimator.addObjectivePosition(10.0, 0.0, x0);
      estimator.addObjectiveVelocity(10.0, 0.0, v0);
      estimator.addObjectivePosition(10.0, duration, xf);
      estimator.addObjectiveVelocity(10.0, duration, vf);

      estimator.solve();

      assertEquals(4, estimator.getOrder());

      coefficients = estimator.getCoefficients();
      assertEquals(c0, coefficients.get(0, 0), epsilon);
      assertEquals(c1, coefficients.get(1, 0), epsilon);
      assertEquals(c2, coefficients.get(2, 0), epsilon);
      assertEquals(c3, coefficients.get(3, 0), epsilon);

      estimator.compute(0.0);
      assertEquals(x0, estimator.getPosition(), epsilon);
      assertEquals(v0, estimator.getVelocity(), epsilon);
      assertEquals(a0, estimator.getAcceleration(), epsilon);

      estimator.compute(2.0);
      assertEquals(xf, estimator.getPosition(), epsilon);
      assertEquals(vf, estimator.getVelocity(), epsilon);
      assertEquals(af, estimator.getAcceleration(), epsilon);

      // do the same test, but with some acceleration at the beginning
      estimator.reset();

      estimator.addObjectivePosition(0.0, x0);
      estimator.addObjectiveVelocity(0.0, v0);
      estimator.addObjectiveAcceleration(0.0, a0);
      estimator.addObjectivePosition(duration, xf);

      estimator.solve();

      assertEquals(4, estimator.getOrder());

      coefficients = estimator.getCoefficients();
      assertEquals(c0, coefficients.get(0, 0), epsilon);
      assertEquals(c1, coefficients.get(1, 0), epsilon);
      assertEquals(c2, coefficients.get(2, 0), epsilon);
      assertEquals(c3, coefficients.get(3, 0), epsilon);

      estimator.compute(0.0);
      assertEquals(x0, estimator.getPosition(), epsilon);
      assertEquals(v0, estimator.getVelocity(), epsilon);
      assertEquals(a0, estimator.getAcceleration(), epsilon);

      estimator.compute(2.0);
      assertEquals(xf, estimator.getPosition(), epsilon);
      assertEquals(vf, estimator.getVelocity(), epsilon);
      assertEquals(af, estimator.getAcceleration(), epsilon);

      // do the same test, but with some acceleration at the end
      estimator.reset();

      estimator.addObjectivePosition(0.0, x0);
      estimator.addObjectiveVelocity(0.0, v0);
      estimator.addObjectivePosition(duration, xf);
      estimator.addObjectiveAcceleration(duration, af);

      estimator.solve();

      assertEquals(4, estimator.getOrder());

      coefficients = estimator.getCoefficients();
      assertEquals(c0, coefficients.get(0, 0), epsilon);
      assertEquals(c1, coefficients.get(1, 0), epsilon);
      assertEquals(c2, coefficients.get(2, 0), epsilon);
      assertEquals(c3, coefficients.get(3, 0), epsilon);

      estimator.compute(0.0);
      assertEquals(x0, estimator.getPosition(), epsilon);
      assertEquals(v0, estimator.getVelocity(), epsilon);
      assertEquals(a0, estimator.getAcceleration(), epsilon);

      estimator.compute(2.0);
      assertEquals(xf, estimator.getPosition(), epsilon);
      assertEquals(vf, estimator.getVelocity(), epsilon);
      assertEquals(af, estimator.getAcceleration(), epsilon);
   }

   @Test
   public void testCubicWithInitialConstraints()
   {
      PolynomialEstimator unboundedEstimator = new PolynomialEstimator();
      PolynomialEstimator boundedEstimator = new PolynomialEstimator();
      unboundedEstimator.reshape(4);
      boundedEstimator.reshape(7);

      double duration = 2.0;
      double x0 = 2.0;
      double xf = 4.0;
      double v0 = 2.0;
      double vf = -2.0;
      unboundedEstimator.addObjectivePosition(0.0, x0);
      unboundedEstimator.addObjectiveVelocity(0.0, v0);
      unboundedEstimator.addObjectivePosition(duration, xf);
      unboundedEstimator.addObjectiveVelocity(duration, vf);

      unboundedEstimator.solve();

      double c2 = 3.0 / (duration * duration) * (xf - x0) - 1.0 / duration * (2.0 * v0 + vf);
      double c3 = 2.0 / (duration * duration * duration) * (x0 - xf) + 1.0 / (duration * duration) * (v0 + vf);

      double a0 = 2.0 * c2;
      double af = a0 + 6.0 * duration * c3;

      x0 = 3.0;
      v0 = 2.0;

      boundedEstimator.addConstraintPosition(0.0, x0);
      boundedEstimator.addConstraintVelocity(0.0, v0);
      boundedEstimator.addConstraintAcceleration(0.0, a0);

      for (double time = 0.0; time < duration; time += 0.005)
      {
         unboundedEstimator.compute(time);
         boundedEstimator.addObjectivePosition(time, unboundedEstimator.getPosition());
      }

         unboundedEstimator.compute(duration);
         boundedEstimator.addObjectivePosition(100.0, duration, unboundedEstimator.getPosition());

      boundedEstimator.solve();

      boundedEstimator.compute(0.0);
      assertEquals(x0, boundedEstimator.getPosition(), epsilon);
      assertEquals(v0, boundedEstimator.getVelocity(), epsilon);
      assertEquals(a0, boundedEstimator.getAcceleration(), epsilon);

      boundedEstimator.compute(duration);
      assertEquals(xf, boundedEstimator.getPosition(), 1e-1);
   }

   @Test
   public void testQuartic()
   {
      PolynomialEstimator estimator = new PolynomialEstimator();
      estimator.reshape(5);

      double duration = 2.0;
      double x0 = 2.0;
      double xf = 4.0;
      double v0 = 2.0;
      double vf = -2.0;
      estimator.addObjectivePosition(0.0, x0);
      estimator.addObjectiveVelocity(0.0, v0);
      estimator.addObjectivePosition(duration, xf);
      estimator.addObjectiveVelocity(duration, vf);

      estimator.solve();

      assertEquals(5, estimator.getOrder());

      DMatrixRMaj coefficients = estimator.getCoefficients();
      assertEquals(5, coefficients.getNumRows());
      assertEquals(1, coefficients.getNumCols());

      estimator.compute(0.0);
      assertEquals(x0, estimator.getPosition(), epsilon);
      assertEquals(v0, estimator.getVelocity(), epsilon);

      estimator.compute(2.0);
      assertEquals(xf, estimator.getPosition(), epsilon);
      assertEquals(vf, estimator.getVelocity(), epsilon);

   }
}
