package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RateLimitedYoFrameOrientationTest
{
   private static final double EPSILON = 1.0e-11;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testConvergenceWithConstantInput() throws Exception
   {
      Random random = new Random(46363);
      double dt = 0.004;
      MutableDouble maxRate = new MutableDouble();
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      RateLimitedYoFrameOrientation rateLimitedOrientation = new RateLimitedYoFrameOrientation("blop", "", registry,
                                                                                               (DoubleProvider) () -> maxRate.doubleValue(), dt,
                                                                                               ReferenceFrame.getWorldFrame());
      rateLimitedOrientation.update(new Quaternion());

      FiniteDifferenceAngularVelocityYoFrameVector angularVelocity = new FiniteDifferenceAngularVelocityYoFrameVector("rate", ReferenceFrame.getWorldFrame(),
                                                                                                                      dt, registry);

      for (int i = 0; i < 1000; i++)
      {
         maxRate.setValue(RandomNumbers.nextDouble(random, 0.001, 10.0));
         Quaternion goalQuaternion = EuclidCoreRandomTools.nextQuaternion(random);

         double distanceToGoal = Math.abs(AngleTools.trimAngleMinusPiToPi(new Quaternion(rateLimitedOrientation).distance(goalQuaternion)));
         if (distanceToGoal / dt < maxRate.doubleValue())
         { // Should converge in one step
            rateLimitedOrientation.update(goalQuaternion);
            EuclidCoreTestTools.assertQuaternionGeometricallyEquals(goalQuaternion, new Quaternion(rateLimitedOrientation), EPSILON);
         }
         else
         {
            double timeToConverge = distanceToGoal / maxRate.doubleValue();
            int numberOfIterations = (int) (timeToConverge / dt);
            double previousDistance = distanceToGoal;
            angularVelocity.update(rateLimitedOrientation);

            for (int j = 0; j < numberOfIterations; j++)
            {
               rateLimitedOrientation.update(goalQuaternion);
               double distance = Math.abs(AngleTools.trimAngleMinusPiToPi(new Quaternion(rateLimitedOrientation).distance(goalQuaternion)));
               assertTrue(distance < previousDistance);
               angularVelocity.update(rateLimitedOrientation);
               double rate = angularVelocity.length();
               assertEquals("difference: " + Math.abs(rate - maxRate.doubleValue()), rate, maxRate.doubleValue(), EPSILON);
               previousDistance = distance;
               assertFalse(new Quaternion(rateLimitedOrientation).geometricallyEquals(goalQuaternion, EPSILON));
            }

            rateLimitedOrientation.update(goalQuaternion);
            EuclidCoreTestTools.assertQuaternionGeometricallyEquals(goalQuaternion, new Quaternion(rateLimitedOrientation), EPSILON);
         }
      }
   }
}
