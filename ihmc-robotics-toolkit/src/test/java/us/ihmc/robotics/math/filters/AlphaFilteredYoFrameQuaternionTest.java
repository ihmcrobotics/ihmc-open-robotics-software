package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AlphaFilteredYoFrameQuaternionTest
{

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testInitialValue()
   {
	   MutableDouble alpha = new MutableDouble();
      AlphaFilteredYoFrameQuaternion q = createAlphaFilteredYoFrameQuaternion(() -> alpha.doubleValue());

      // set measurement randomly
      Random random = new Random(12351235L);
      Quaternion qMeasured = RandomGeometry.nextQuaternion(random);
      q.getUnfilteredQuaternion().set(qMeasured);

      // call update once
      q.update();
      Quaternion qFiltered = new Quaternion(q);

      // verify measurement equals filtered
      EuclidCoreTestTools.assertQuaternionEquals(qMeasured, qFiltered, 1e-12);
   }

	@Test
   public void testAlpha1()
   {
      MutableDouble alpha = new MutableDouble();
      AlphaFilteredYoFrameQuaternion q = createAlphaFilteredYoFrameQuaternion(() -> alpha.doubleValue());
      alpha.setValue(1.0);

      Random random = new Random(73464L);

      // update once
      Quaternion qInitial = RandomGeometry.nextQuaternion(random);
      q.getUnfilteredQuaternion().set(qInitial);
      q.update();

      // update 100 more times
      int nUpdates = 100;
      doRandomUpdates(q, random, nUpdates);

      Quaternion qFiltered = new Quaternion(q);

      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(qInitial, qFiltered, 1e-12);
   }

	@Test
   public void testAlpha0()
   {
      MutableDouble alpha = new MutableDouble();
      AlphaFilteredYoFrameQuaternion q = createAlphaFilteredYoFrameQuaternion(() -> alpha.doubleValue());
      alpha.setValue(0.0);
      
      Random random = new Random(12525123L);

      // update 100 times
      int nUpdates = 100;
      doRandomUpdates(q, random, nUpdates);

      // update one more time
      Quaternion qFinal = RandomGeometry.nextQuaternion(random);
      q.getUnfilteredQuaternion().set(qFinal);
      q.update();

      Quaternion qFiltered = new Quaternion(q);

      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(qFinal, qFiltered, 1e-12);
   }

	@Test
   public void testStepChange()
   {
      MutableDouble alpha = new MutableDouble();
      AlphaFilteredYoFrameQuaternion q = createAlphaFilteredYoFrameQuaternion(() -> alpha.doubleValue());
      alpha.setValue(0.5);

      Random random = new Random(12525123L);

      // update once
      Quaternion qInitial = RandomGeometry.nextQuaternion(random);
      q.getUnfilteredQuaternion().set(qInitial);
      q.update();

      // update a whole bunch of times using the same quaternion
      Quaternion qFinal = RandomGeometry.nextQuaternion(random);
      q.getUnfilteredQuaternion().set(qFinal);

      double angleDifference = getAngleDifference(qInitial, qFinal);
      double epsilon = 1e-3;

      int nUpdates = 100;
      Quaternion qFiltered = new Quaternion();
      for (int i = 0; i < nUpdates; i++)
      {
         q.update();
         qFiltered.set(q);
         double newAngleDifference = getAngleDifference(qFiltered, qFinal);
         //         System.out.println(i + ": " + newAngleDifference);
         boolean sameQuaternion = newAngleDifference == 0.0;
         assertTrue(sameQuaternion || newAngleDifference < (1.0 + epsilon) * alpha.doubleValue() * angleDifference);
         angleDifference = newAngleDifference;
      }
   }

   private void doRandomUpdates(AlphaFilteredYoFrameQuaternion q, Random random, int nUpdates)
   {
      for (int i = 0; i < nUpdates; i++)
      {
         // set measurement randomly and updated filtered version
         Quaternion qMeasured = RandomGeometry.nextQuaternion(random);
         q.getUnfilteredQuaternion().set(qMeasured);
         q.update();
      }
   }

   private AlphaFilteredYoFrameQuaternion createAlphaFilteredYoFrameQuaternion(DoubleProvider alpha)
   {
      YoRegistry registry = new YoRegistry("test");
      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      YoFrameQuaternion unfilteredQuaternion = new YoFrameQuaternion("qMeasured", referenceFrame, registry);
      AlphaFilteredYoFrameQuaternion q = new AlphaFilteredYoFrameQuaternion("qFiltered", "", unfilteredQuaternion, alpha, registry);
      return q;
   }

   private static double getAngleDifference(Quaternion q1, Quaternion q2)
   {
      Quaternion qDifference = new Quaternion(q1);
      qDifference.multiplyConjugateOther(q2);
      AxisAngle axisAngle = new AxisAngle();
      axisAngle.set(qDifference);
      return axisAngle.getAngle();
   }
}
