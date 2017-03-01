package us.ihmc.robotics.kinematics;

import static org.junit.Assert.assertArrayEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.random.RandomGeometry;

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
         Quaternion quat1 = RandomGeometry.nextQuaternion(random);
         Quaternion quat2 = RandomGeometry.nextQuaternion(random);
         Quaternion expectedAverageQuat = new Quaternion();
         expectedAverageQuat.interpolate(quat1, quat2, 0.5);

         AverageQuaternionCalculator averageQuaternionCalculator = new AverageQuaternionCalculator();
         averageQuaternionCalculator.queueQuaternion(quat1);
         averageQuaternionCalculator.queueQuaternion(quat2);
         averageQuaternionCalculator.compute();
         Quaternion actualAverageQuat = new Quaternion();
         averageQuaternionCalculator.getAverageQuaternion(actualAverageQuat);

         if (expectedAverageQuat.getS() * actualAverageQuat.getS() < 0.0)
            expectedAverageQuat.negate();

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
         Vector3D randomRotationAxis = RandomGeometry.nextVector3D(random, 1.0);
         double[] randomAngles = RandomNumbers.nextDoubleArray(random, 100, Math.PI);

         AxisAngle expectedAverageAxisAngle = new AxisAngle(randomRotationAxis, AngleTools.computeAngleAverage(randomAngles));
         Quaternion expectedAverageQuat = new Quaternion();
         expectedAverageQuat.set(expectedAverageAxisAngle);

         AverageQuaternionCalculator averageQuaternionCalculator = new AverageQuaternionCalculator();
         for (int i = 0; i < randomAngles.length; i++)
         {
            AxisAngle tempAxisAngle = new AxisAngle(randomRotationAxis, randomAngles[i]);
            Quaternion tempQuat4d = new Quaternion();
            tempQuat4d.set(tempAxisAngle);
            averageQuaternionCalculator.queueQuaternion(tempQuat4d);
         }
         averageQuaternionCalculator.compute();
         Quaternion actualAverageQuat = new Quaternion();
         averageQuaternionCalculator.getAverageQuaternion(actualAverageQuat);

         if (expectedAverageQuat.getS() * actualAverageQuat.getS() < 0.0)
            expectedAverageQuat.negate();

         double[] expecteds = new double[4];
         expectedAverageQuat.get(expecteds);
         double[] actuals = new double[4];
         actualAverageQuat.get(actuals);
         assertArrayEquals(expecteds, actuals, epsilon);
      }
   }
}
