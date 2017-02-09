package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class AlphaFilteredYoFrameQuaternionTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testInitialValue()
   {
      AlphaFilteredYoFrameQuaternion q = createAlphaFilteredYoFrameQuaternion();

      // set measurement randomly
      Random random = new Random(12351235L);
      Quat4d qMeasured = RandomTools.generateRandomQuaternion(random);
      q.getUnfilteredQuaternion().set(qMeasured);

      // call update once
      q.update();
      Quat4d qFiltered = new Quat4d();
      q.get(qFiltered);

      // verify measurement equals filtered
      JUnitTools.assertQuaternionsEqual(qMeasured, qFiltered, 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAlpha1()
   {
      AlphaFilteredYoFrameQuaternion q = createAlphaFilteredYoFrameQuaternion();
      q.setAlpha(1.0);

      Random random = new Random(73464L);

      // update once
      Quat4d qInitial = RandomTools.generateRandomQuaternion(random);
      q.getUnfilteredQuaternion().set(qInitial);
      q.update();

      // update 100 more times
      int nUpdates = 100;
      doRandomUpdates(q, random, nUpdates);

      Quat4d qFiltered = new Quat4d();
      q.get(qFiltered);

      JUnitTools.assertQuaternionsEqualUsingDifference(qInitial, qFiltered, 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAlpha0()
   {
      AlphaFilteredYoFrameQuaternion q = createAlphaFilteredYoFrameQuaternion();
      q.setAlpha(0.0);

      Random random = new Random(12525123L);

      // update 100 times
      int nUpdates = 100;
      doRandomUpdates(q, random, nUpdates);

      // update one more time
      Quat4d qFinal = RandomTools.generateRandomQuaternion(random);
      q.getUnfilteredQuaternion().set(qFinal);
      q.update();

      Quat4d qFiltered = new Quat4d();
      q.get(qFiltered);

      JUnitTools.assertQuaternionsEqualUsingDifference(qFinal, qFiltered, 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testStepChange()
   {
      AlphaFilteredYoFrameQuaternion q = createAlphaFilteredYoFrameQuaternion();
      double alpha = 0.5;
      q.setAlpha(alpha);

      Random random = new Random(12525123L);

      // update once
      Quat4d qInitial = RandomTools.generateRandomQuaternion(random);
      q.getUnfilteredQuaternion().set(qInitial);
      q.update();

      // update a whole bunch of times using the same quaternion
      Quat4d qFinal = RandomTools.generateRandomQuaternion(random);
      q.getUnfilteredQuaternion().set(qFinal);

      double angleDifference = getAngleDifference(qInitial, qFinal);
      double epsilon = 1e-3;

      int nUpdates = 100;
      Quat4d qFiltered = new Quat4d();
      for (int i = 0; i < nUpdates; i++)
      {
         q.update();
         q.get(qFiltered);
         double newAngleDifference = getAngleDifference(qFiltered, qFinal);
         //         System.out.println(i + ": " + newAngleDifference);
         boolean sameQuaternion = newAngleDifference == 0.0;
         assertTrue(sameQuaternion || newAngleDifference < (1.0 + epsilon) * alpha * angleDifference);
         angleDifference = newAngleDifference;
      }
   }

   private void doRandomUpdates(AlphaFilteredYoFrameQuaternion q, Random random, int nUpdates)
   {
      for (int i = 0; i < nUpdates; i++)
      {
         // set measurement randomly and updated filtered version
         Quat4d qMeasured = RandomTools.generateRandomQuaternion(random);
         q.getUnfilteredQuaternion().set(qMeasured);
         q.update();
      }
   }

   private AlphaFilteredYoFrameQuaternion createAlphaFilteredYoFrameQuaternion()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      YoFrameQuaternion unfilteredQuaternion = new YoFrameQuaternion("qMeasured", referenceFrame, registry);
      double alpha = 0.0;
      AlphaFilteredYoFrameQuaternion q = new AlphaFilteredYoFrameQuaternion("qFiltered", "", unfilteredQuaternion, alpha, registry);
      return q;
   }

   private static double getAngleDifference(Quat4d q1, Quat4d q2)
   {
      Quat4d qDifference = new Quat4d();
      qDifference.mulInverse(q1, q2);
      AxisAngle4d axisAngle = new AxisAngle4d();
      axisAngle.set(qDifference);
      return axisAngle.getAngle();
   }
}
