package us.ihmc.robotics.math.interpolators;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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

         Matrix3D interpolatedOrientationMatrixDot = new Matrix3D();
         RigidBodyTransform transformationAtDt = new RigidBodyTransform();
         interpolatedOrientationDt.getTransform3D(transformationAtDt);
         transformationAtDt.getRotation(interpolatedOrientationMatrixDot);
         RotationMatrix interpolatedOrientation0Matrix = new RotationMatrix();
         RigidBodyTransform transformationAt0 = new RigidBodyTransform();
         interpolatedOrientation0.getTransform3D(transformationAt0);
         transformationAt0.getRotation(interpolatedOrientation0Matrix);
         interpolatedOrientationMatrixDot.sub(interpolatedOrientation0Matrix);
         interpolatedOrientationMatrixDot.scale(1.0 / dt);

         RotationMatrix orientationTranspose = new RotationMatrix(interpolatedOrientation0Matrix);
         orientationTranspose.transpose();
         Matrix3D omegaTilde = new Matrix3D(interpolatedOrientationMatrixDot);
         omegaTilde.multiply(orientationTranspose);
         assertTrue(omegaTilde.isMatrixSkewSymmetric(1.0e-3));

         Vector3D angularVelocity = new Vector3D(-omegaTilde.getM12(), omegaTilde.getM02(), -omegaTilde.getM01()); // in world frame

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
