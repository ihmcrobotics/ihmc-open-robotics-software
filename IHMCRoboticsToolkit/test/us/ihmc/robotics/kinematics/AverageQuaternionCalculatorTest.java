package us.ihmc.robotics.kinematics;

import static org.junit.Assert.assertArrayEquals;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.random.RandomTools;

public class AverageQuaternionCalculatorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAgainstInterpolation() throws Exception
   {
      for (int nTest = 0; nTest < 10; nTest++)
      {
         double epsilon = 1.0e-15;
         Random random = new Random(56416456L);
         Quat4d quat1 = RandomTools.generateRandomQuaternion(random);
         Quat4d quat2 = RandomTools.generateRandomQuaternion(random);
         Quat4d expectedAverageQuat = new Quat4d();
         expectedAverageQuat.interpolate(quat1, quat2, 0.5);

         AverageQuaternionCalculator averageQuaternionCalculator = new AverageQuaternionCalculator();
         averageQuaternionCalculator.queueQuaternion(quat1);
         averageQuaternionCalculator.queueQuaternion(quat2);
         averageQuaternionCalculator.compute();
         Quat4d actualAverageQuat = new Quat4d();
         averageQuaternionCalculator.getAverageQuaternion(actualAverageQuat);

         if (expectedAverageQuat.getW() * actualAverageQuat.getW() < 0.0)
            expectedAverageQuat.scale(-1.0);

         double[] expecteds = new double[4];
         expectedAverageQuat.get(expecteds);
         double[] actuals = new double[4];
         actualAverageQuat.get(actuals);
         assertArrayEquals(expecteds, actuals, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWithRotationsAroundTheSameAxis() throws Exception
   {
      for (int nTest = 0; nTest < 10; nTest++)
      {
         double epsilon = 1.0e-15;
         Random random = new Random(56416456L);
         Vector3d randomRotationAxis = RandomTools.generateRandomVector(random, 1.0);
         double[] randomAngles = RandomTools.generateRandomDoubleArray(random, 100, Math.PI);

         AxisAngle4d expectedAverageAxisAngle = new AxisAngle4d(randomRotationAxis, AngleTools.computeAngleAverage(randomAngles));
         Quat4d expectedAverageQuat = new Quat4d();
         expectedAverageQuat.set(expectedAverageAxisAngle);

         AverageQuaternionCalculator averageQuaternionCalculator = new AverageQuaternionCalculator();
         for (int i = 0; i < randomAngles.length; i++)
         {
            AxisAngle4d tempAxisAngle = new AxisAngle4d(randomRotationAxis, randomAngles[i]);
            Quat4d tempQuat4d = new Quat4d();
            tempQuat4d.set(tempAxisAngle);
            averageQuaternionCalculator.queueQuaternion(tempQuat4d);
         }
         averageQuaternionCalculator.compute();
         Quat4d actualAverageQuat = new Quat4d();
         averageQuaternionCalculator.getAverageQuaternion(actualAverageQuat);

         if (expectedAverageQuat.getW() * actualAverageQuat.getW() < 0.0)
            expectedAverageQuat.scale(-1.0);

         double[] expecteds = new double[4];
         expectedAverageQuat.get(expecteds);
         double[] actuals = new double[4];
         actualAverageQuat.get(actuals);
         assertArrayEquals(expecteds, actuals, epsilon);
      }
   }
}
