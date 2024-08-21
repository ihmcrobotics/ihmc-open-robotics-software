package us.ihmc.robotics.math.frames;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.*;

public class YoFrameQuaternionTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double EPS = 1e-8;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testInitialization()
   {
      YoRegistry registry = new YoRegistry("blop");
      YoFrameQuaternion yoFrameQuaternion = new YoFrameQuaternion("test", worldFrame, registry);

      yoFrameQuaternion.checkReferenceFrameMatch(worldFrame);

      Quaternion quat4dActual = new Quaternion(yoFrameQuaternion);
      Quaternion quat4dExpected = new Quaternion(0.0, 0.0, 0.0, 1.0);
      EuclidCoreTestTools.assertEquals(quat4dExpected, quat4dActual, EPS);

      AxisAngle axisAngle4dActual = new AxisAngle(yoFrameQuaternion);
      AxisAngle axisAngle4dExpected = new AxisAngle(1.0, 0.0, 0.0, 0.0);
      EuclidCoreTestTools.assertEquals(axisAngle4dExpected, axisAngle4dActual, EPS);

      RotationMatrix matrix3dActual = new RotationMatrix(yoFrameQuaternion);
      RotationMatrix matrix3dExpected = new RotationMatrix();
      matrix3dExpected.setIdentity();
      EuclidCoreTestTools.assertEquals(matrix3dExpected, matrix3dActual, EPS);

      FrameQuaternion frameOrientationActual = new FrameQuaternion(yoFrameQuaternion);
      FrameQuaternion frameOrientationExpected = new FrameQuaternion(worldFrame);
      EuclidFrameTestTools.assertEquals(frameOrientationExpected, frameOrientationActual, EPS);

      YawPitchRoll yawPitchRollActual = new YawPitchRoll(yoFrameQuaternion);
      YawPitchRoll yawPitchRollExpected = new YawPitchRoll();
      EuclidCoreTestTools.assertEquals(yawPitchRollExpected, yawPitchRollActual, EPS);
   }

   @Test
   public void testSetters()
   {
      Random random = new Random(1972L);

      YoRegistry registry = new YoRegistry("blop");
      YoFrameQuaternion yoFrameQuaternion = new YoFrameQuaternion("test", worldFrame, registry);

      RotationMatrix matrix3dExpected = new RotationMatrix();
      RotationMatrix matrix3dActual = new RotationMatrix();

      Quaternion quat4dExpected = EuclidCoreRandomTools.nextQuaternion(random);
      yoFrameQuaternion.set(quat4dExpected);
      Quaternion quat4dActual = new Quaternion(yoFrameQuaternion);
      EuclidCoreTestTools.assertEquals(quat4dExpected, quat4dActual, EPS);

      AxisAngle axisAngle4dExpected = EuclidCoreRandomTools.nextAxisAngle(random);
      yoFrameQuaternion.set(axisAngle4dExpected);
      AxisAngle axisAngle4dActual = new AxisAngle(yoFrameQuaternion);
      assertTrue(RotationTools.axisAngleEpsilonEqualsIgnoreFlippedAxes(axisAngle4dExpected, axisAngle4dActual, EPS));

      matrix3dExpected.set(EuclidCoreRandomTools.nextAxisAngle(random));
      yoFrameQuaternion.set(matrix3dExpected);
      matrix3dActual.set(yoFrameQuaternion);
      EuclidCoreTestTools.assertGeometricallyEquals(matrix3dExpected, matrix3dActual, EPS);

      FrameQuaternion frameOrientationExpected = new FrameQuaternion(worldFrame);
      frameOrientationExpected.set(EuclidCoreRandomTools.nextQuaternion(random));
      yoFrameQuaternion.set(frameOrientationExpected);
      FrameQuaternion frameOrientationActual = new FrameQuaternion(yoFrameQuaternion);
      EuclidFrameTestTools.assertEquals(frameOrientationExpected, frameOrientationActual, EPS);

      YawPitchRoll yawPitchRollExpected = EuclidCoreRandomTools.nextYawPitchRoll(random);
      yoFrameQuaternion.set(yawPitchRollExpected);
      YawPitchRoll yawPitchRollActual = new YawPitchRoll();
      yawPitchRollActual.set(yoFrameQuaternion);

      matrix3dActual.set(yawPitchRollActual);
      matrix3dExpected.set(yawPitchRollExpected);

      EuclidCoreTestTools.assertEquals(matrix3dExpected, matrix3dActual, EPS);
   }

   @Test
   public void testReferenceFramesMismatching()
   {
      Random random = new Random(1984L);
      ReferenceFrame testFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("chou", worldFrame, EuclidCoreRandomTools.nextRigidBodyTransform(random));

      YoRegistry registry = new YoRegistry("blop");
      YoFrameQuaternion yoFrameQuaternion = new YoFrameQuaternion("test", worldFrame, registry);

      FrameQuaternion frameOrientation = new FrameQuaternion(testFrame);
      boolean hasReferenceFrameMismatchExceptionBeenThrown = false;
      try
      {
         yoFrameQuaternion.set(frameOrientation);
      }
      catch (ReferenceFrameMismatchException e)
      {
         hasReferenceFrameMismatchExceptionBeenThrown = true;
      }
      assertTrue(hasReferenceFrameMismatchExceptionBeenThrown);

      frameOrientation = new FrameQuaternion(worldFrame);
      hasReferenceFrameMismatchExceptionBeenThrown = false;
      try
      {
         yoFrameQuaternion.set(frameOrientation);
      }
      catch (ReferenceFrameMismatchException e)
      {
         hasReferenceFrameMismatchExceptionBeenThrown = true;
      }
      assertFalse(hasReferenceFrameMismatchExceptionBeenThrown);
   }

   @Test
   public void testMultiplication()
   {
      Random random = new Random(1776L);

      YoRegistry registry = new YoRegistry("blop");
      YoFrameQuaternion yoFrameQuaternion = new YoFrameQuaternion("test", worldFrame, registry);

      Quaternion quat4dActual = new Quaternion(), quat4dExpected = new Quaternion();
      Quaternion quat4dA, quat4dB;

      FrameQuaternion frameOrientation = new FrameQuaternion(worldFrame);

      for (int i = 0; i < 1000; i++)
      {
         quat4dA = EuclidCoreRandomTools.nextQuaternion(random);
         quat4dB = EuclidCoreRandomTools.nextQuaternion(random);
         quat4dExpected.multiply(quat4dA, quat4dB);

         yoFrameQuaternion.set(quat4dA);
         yoFrameQuaternion.multiply(quat4dB);
         quat4dActual.set(yoFrameQuaternion);
         EuclidCoreTestTools.assertEquals(quat4dExpected, quat4dActual, EPS);

         yoFrameQuaternion.set(quat4dA);
         frameOrientation.set(quat4dB);
         yoFrameQuaternion.multiply(frameOrientation);
         quat4dActual.set(yoFrameQuaternion);
         EuclidCoreTestTools.assertEquals(quat4dExpected, quat4dActual, EPS);
      }
   }

   @Test
   public void testInterpolate()
   {
      Random random = new Random(1776L);

      YoRegistry registry = new YoRegistry("blop");

      FrameQuaternion initialFrameOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalFrameOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion interpolatedFrameOrientation = new FrameQuaternion(worldFrame);

      YoFrameQuaternion initialYoFrameQuaternion = new YoFrameQuaternion("init", worldFrame, registry);
      initialYoFrameQuaternion.set(initialFrameOrientation);
      YoFrameQuaternion finalYoFrameQuaternion = new YoFrameQuaternion("final", worldFrame, registry);
      finalYoFrameQuaternion.set(finalFrameOrientation);
      YoFrameQuaternion interpolatedYoFrameQuaternion = new YoFrameQuaternion("interpolated", worldFrame, registry);

      FrameQuaternion temp = new FrameQuaternion();
      for (double alpha = -0.1; alpha <= 1.1; alpha += 0.05)
      {
         interpolatedFrameOrientation.interpolate(initialFrameOrientation, finalFrameOrientation, alpha);
         interpolatedYoFrameQuaternion.interpolate(initialYoFrameQuaternion, finalYoFrameQuaternion, alpha);
         temp.setIncludingFrame(interpolatedYoFrameQuaternion);

         EuclidFrameTestTools.assertEquals(interpolatedFrameOrientation, temp, EPS);
      }
   }
}
