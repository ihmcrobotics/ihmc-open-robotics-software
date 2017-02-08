package us.ihmc.robotics.math.frames;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameQuaternionTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double EPS = 1e-8;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testInitialization()
   {
      YoVariableRegistry registry = new YoVariableRegistry("blop");
      YoFrameQuaternion yoFrameQuaternion = new YoFrameQuaternion("test", worldFrame, registry);

      yoFrameQuaternion.checkReferenceFrameMatch(worldFrame);

      Quat4d quat4dActual = new Quat4d();
      yoFrameQuaternion.get(quat4dActual);
      Quat4d quat4dExpected = new Quat4d(0.0, 0.0, 0.0, 1.0);
      assertTrue(quat4dActual.epsilonEquals(quat4dExpected, EPS));

      AxisAngle4d axisAngle4dActual = new AxisAngle4d();
      yoFrameQuaternion.get(axisAngle4dActual);
      AxisAngle4d axisAngle4dExpected = new AxisAngle4d(0.0, 1.0, 0.0, 0.0);
      assertTrue(axisAngle4dActual.epsilonEquals(axisAngle4dExpected, EPS));

      Matrix3d matrix3dActual = new Matrix3d();
      yoFrameQuaternion.get(matrix3dActual);
      Matrix3d matrix3dExpected = new Matrix3d();
      matrix3dExpected.setIdentity();
      assertTrue(matrix3dActual.epsilonEquals(matrix3dExpected, EPS));

      FrameOrientation frameOrientationActual = new FrameOrientation(worldFrame);
      yoFrameQuaternion.getFrameOrientationIncludingFrame(frameOrientationActual);
      FrameOrientation frameOrientationExpected = new FrameOrientation(worldFrame);
      assertTrue(frameOrientationActual.epsilonEquals(frameOrientationExpected, EPS));

      double[] yawPitchRollActual = new double[3];
      yoFrameQuaternion.getYawPitchRoll(yawPitchRollActual);
      double[] yawPitchRollExpected = new double[3];
      assertArrayEquals(yawPitchRollExpected, yawPitchRollActual, EPS);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetters()
   {
      Random random = new Random(1972L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      YoFrameQuaternion yoFrameQuaternion = new YoFrameQuaternion("test", worldFrame, registry);

      Matrix3d matrix3dExpected = new Matrix3d();
      Matrix3d matrix3dActual = new Matrix3d();

      Quat4d quat4dExpected = RandomTools.generateRandomQuaternion(random);
      yoFrameQuaternion.set(quat4dExpected);
      Quat4d quat4dActual = new Quat4d();
      yoFrameQuaternion.get(quat4dActual);
      assertTrue(RotationTools.quaternionEpsilonEquals(quat4dExpected, quat4dActual, EPS));

      AxisAngle4d axisAngle4dExpected = RandomTools.generateRandomRotation(random);
      yoFrameQuaternion.set(axisAngle4dExpected);
      AxisAngle4d axisAngle4dActual = new AxisAngle4d();
      yoFrameQuaternion.get(axisAngle4dActual);
      assertTrue(RotationTools.axisAngleEpsilonEqualsIgnoreFlippedAxes(axisAngle4dExpected, axisAngle4dActual, EPS));

      matrix3dExpected.set(RandomTools.generateRandomRotation(random));
      yoFrameQuaternion.set(matrix3dExpected);
      yoFrameQuaternion.get(matrix3dActual);
      assertTrue(matrix3dActual.epsilonEquals(matrix3dExpected, EPS));

      FrameOrientation frameOrientationExpected = new FrameOrientation(worldFrame);
      frameOrientationExpected.set(RandomTools.generateRandomQuaternion(random));
      yoFrameQuaternion.set(frameOrientationExpected);
      FrameOrientation frameOrientationActual = new FrameOrientation(worldFrame);
      yoFrameQuaternion.getFrameOrientationIncludingFrame(frameOrientationActual);
      assertTrue(frameOrientationActual.epsilonEquals(frameOrientationExpected, EPS));

      double[] yawPitchRollExpected = RandomTools.generateRandomDoubleArray(random, 3, 2.0 * Math.PI);
      yoFrameQuaternion.set(yawPitchRollExpected[0], yawPitchRollExpected[1], yawPitchRollExpected[2]);
      double[] yawPitchRollActual = new double[3];
      yoFrameQuaternion.getYawPitchRoll(yawPitchRollActual);

      RotationTools.convertYawPitchRollToMatrix(yawPitchRollActual[0], yawPitchRollActual[1], yawPitchRollActual[2], matrix3dActual);
      RotationTools.convertYawPitchRollToMatrix(yawPitchRollExpected[0], yawPitchRollExpected[1], yawPitchRollExpected[2], matrix3dExpected);

      assertTrue(matrix3dActual.epsilonEquals(matrix3dExpected, EPS));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testReferenceFramesMismatching()
   {
      Random random = new Random(1984L);
      ReferenceFrame testFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("chou", worldFrame, RigidBodyTransform
            .generateRandomTransform(random));

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      YoFrameQuaternion yoFrameQuaternion = new YoFrameQuaternion("test", worldFrame, registry);

      FrameOrientation frameOrientation = new FrameOrientation(testFrame);
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

      frameOrientation = new FrameOrientation(worldFrame);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMultiplication()
   {
      Random random = new Random(1776L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");
      YoFrameQuaternion yoFrameQuaternion = new YoFrameQuaternion("test", worldFrame, registry);

      Quat4d quat4dActual = new Quat4d(), quat4dExpected = new Quat4d();
      Quat4d quat4dA, quat4dB;

      FrameOrientation frameOrientation = new FrameOrientation(worldFrame);

      for (int i = 0; i < 1000; i++)
      {
         quat4dA = RandomTools.generateRandomQuaternion(random);
         quat4dB = RandomTools.generateRandomQuaternion(random);
         quat4dExpected.mul(quat4dA, quat4dB);

         yoFrameQuaternion.set(quat4dA);
         yoFrameQuaternion.mul(quat4dB);
         yoFrameQuaternion.get(quat4dActual);
         assertTrue(RotationTools.quaternionEpsilonEquals(quat4dExpected, quat4dActual, EPS));

         yoFrameQuaternion.set(quat4dA);
         frameOrientation.set(quat4dB);
         yoFrameQuaternion.mul(frameOrientation);
         yoFrameQuaternion.get(quat4dActual);
         assertTrue(RotationTools.quaternionEpsilonEquals(quat4dExpected, quat4dActual, EPS));

      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testInterpolate()
   {
      Random random = new Random(1776L);

      YoVariableRegistry registry = new YoVariableRegistry("blop");

      FrameOrientation initialFrameOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      FrameOrientation finalFrameOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      FrameOrientation interpolatedFrameOrientation = new FrameOrientation(worldFrame);

      YoFrameQuaternion initialYoFrameQuaternion = new YoFrameQuaternion("init", worldFrame, registry);
      initialYoFrameQuaternion.set(initialFrameOrientation);
      YoFrameQuaternion finalYoFrameQuaternion = new YoFrameQuaternion("final", worldFrame, registry);
      finalYoFrameQuaternion.set(finalFrameOrientation);
      YoFrameQuaternion interpolatedYoFrameQuaternion = new YoFrameQuaternion("interpolated", worldFrame, registry);

      FrameOrientation temp = new FrameOrientation();
      for (double alpha = -0.1; alpha <= 1.1; alpha += 0.05)
      {
         interpolatedFrameOrientation.interpolate(initialFrameOrientation, finalFrameOrientation, alpha);
         interpolatedYoFrameQuaternion.interpolate(initialYoFrameQuaternion, finalYoFrameQuaternion, alpha);
         interpolatedYoFrameQuaternion.getFrameOrientationIncludingFrame(temp);

         assertTrue(interpolatedFrameOrientation.epsilonEquals(temp, EPS));
      }
   }
}
