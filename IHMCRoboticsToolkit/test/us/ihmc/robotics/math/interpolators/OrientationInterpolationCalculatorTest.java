package us.ihmc.robotics.math.interpolators;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class OrientationInterpolationCalculatorTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)

   /**
    * Numerically differentiate, then use OrientationInterpolationAngularVelocityCalculator, compare results.
    */
   public void testComputeAngularVelocity()
   {
      Random random = new Random(101L);

      ReferenceFrame world = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < 50000; i++)
      {
         FrameOrientation startOrientation = FrameOrientation.generateRandomFrameOrientation(random, world);
         FrameOrientation endOrientation = FrameOrientation.generateRandomFrameOrientation(random, world);

         double alpha = random.nextDouble();
         double alphaDot = random.nextDouble();
         double dt = 1e-6;
         double deltaAlpha = alphaDot * dt;
         double alphaNew = alpha + deltaAlpha;

         // compute angular velocity by numerical differentiation
         FrameOrientation interpolatedOrientation0 = new FrameOrientation(startOrientation.getReferenceFrame());
         interpolatedOrientation0.interpolate(startOrientation, endOrientation, alpha);

         FrameOrientation interpolatedOrientationDt = new FrameOrientation(startOrientation.getReferenceFrame());
         interpolatedOrientationDt.interpolate(startOrientation, endOrientation, alphaNew);

         Matrix3d interpolatedOrientationMatrixDot = new Matrix3d();
         RigidBodyTransform transformationAtDt = new RigidBodyTransform();
         interpolatedOrientationDt.getTransform3D(transformationAtDt);
         transformationAtDt.getRotation(interpolatedOrientationMatrixDot);
         Matrix3d interpolatedOrientation0Matrix = new Matrix3d();
         RigidBodyTransform transformationAt0 = new RigidBodyTransform();
         interpolatedOrientation0.getTransform3D(transformationAt0);
         transformationAt0.getRotation(interpolatedOrientation0Matrix);
         interpolatedOrientationMatrixDot.sub(interpolatedOrientation0Matrix);
         interpolatedOrientationMatrixDot.mul(1.0 / dt);

         Matrix3d orientationTranspose = new Matrix3d(interpolatedOrientation0Matrix);
         orientationTranspose.transpose();
         Matrix3d omegaTilde = new Matrix3d(interpolatedOrientationMatrixDot);
         omegaTilde.mul(orientationTranspose);
         JUnitTools.assertSkewSymmetric(omegaTilde, 1e-3);

         Vector3d angularVelocity = new Vector3d(-omegaTilde.getM12(), omegaTilde.getM02(), -omegaTilde.getM01()); // in world frame

         // compute angular velocity using OrientationInterpolationAngularVelocityCalculator
         OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();
         FrameVector angularVelocityFromCalculator = new FrameVector();
         orientationInterpolationCalculator.computeAngularVelocity(angularVelocityFromCalculator, startOrientation, endOrientation, alphaDot);

         double epsilon = 1e-5;
         assertEquals(angularVelocity.getX(), angularVelocityFromCalculator.getX(), epsilon);
         assertEquals(angularVelocity.getY(), angularVelocityFromCalculator.getY(), epsilon);
         assertEquals(angularVelocity.getZ(), angularVelocityFromCalculator.getZ(), epsilon);
      }
   }
}
