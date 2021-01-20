package us.ihmc.robotics.math.trajectories;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class PolynomialEstimator3DTest
{
   private static final double epsilon = 5e-4;

   @Test
   public void testLinear()
   {
      PolynomialEstimator3D estimator = new PolynomialEstimator3D();
      estimator.reshape(2);

      double duration = 2.0;
      Point3D startPosition = new Point3D(2.0, 3.0, 4.0);
      Point3D endPosition = new Point3D(7.5, 8.7, 3.2);

      Vector3D velocity = new Vector3D();
      velocity.sub(endPosition, startPosition);
      velocity.scale(1.0 / duration);

      estimator.addObjectivePosition(0.0, startPosition);
      estimator.addObjectivePosition(duration, endPosition);

      estimator.initialize();

      estimator.compute(0.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(startPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, estimator.getVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), estimator.getAcceleration(), epsilon);

      estimator.compute(duration);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(endPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, estimator.getVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), estimator.getAcceleration(), epsilon);

      // do the same test, but with a higher weight
      estimator.reset();

      estimator.addObjectivePosition(10.0, 0.0, startPosition);
      estimator.addObjectivePosition(10.0, duration, endPosition);

      estimator.initialize();

      estimator.compute(0.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(startPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, estimator.getVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), estimator.getAcceleration(), epsilon);

      estimator.compute(duration);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(endPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, estimator.getVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), estimator.getAcceleration(), epsilon);

      // do the same test, but with a velocity guess
      estimator.reset();

      estimator.addObjectivePosition(0.0, startPosition);
      estimator.addObjectiveVelocity(0.0, velocity);

      estimator.initialize();

      estimator.compute(0.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(startPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, estimator.getVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), estimator.getAcceleration(), epsilon);

      estimator.compute(duration);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(endPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, estimator.getVelocity(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new Vector3D(), estimator.getAcceleration(), epsilon);
   }

   @Test
   public void testCubic()
   {
      PolynomialEstimator3D estimator = new PolynomialEstimator3D();
      estimator.reshape(4);

      double duration = 2.0;
      Point3D startPosition = new Point3D(2.0, 3.0, 4.0);
      Point3D endPosition = new Point3D(7.5, 8.7, 3.2);
      Vector3D startVelocity = new Vector3D(6.0, 7.0, 8.0);
      Vector3D endVelocity = new Vector3D(3.5, 4.3, -6.2);


      estimator.addObjectivePosition(0.0, startPosition);
      estimator.addObjectiveVelocity(0.0, startVelocity);
      estimator.addObjectivePosition(duration, endPosition);
      estimator.addObjectiveVelocity(duration, endVelocity);

      estimator.initialize();



      estimator.compute(0.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(startPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(startVelocity, estimator.getVelocity(), epsilon);

      estimator.compute(duration);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(endPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(endVelocity, estimator.getVelocity(), epsilon);

      // do the same test, but with a higher weight
      estimator.reset();

      estimator.addObjectivePosition(10.0, 0.0, startPosition);
      estimator.addObjectiveVelocity(10.0, 0.0, startVelocity);
      estimator.addObjectivePosition(10.0, duration, endPosition);
      estimator.addObjectiveVelocity(10.0, duration, endVelocity);

      estimator.initialize();

      estimator.compute(0.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(startPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(startVelocity, estimator.getVelocity(), epsilon);

      estimator.compute(duration);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(endPosition, estimator.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(endVelocity, estimator.getVelocity(), epsilon);
   }
}
