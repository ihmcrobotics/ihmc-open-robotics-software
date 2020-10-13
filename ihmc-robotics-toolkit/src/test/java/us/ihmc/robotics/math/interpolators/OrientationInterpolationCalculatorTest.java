package us.ihmc.robotics.math.interpolators;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class OrientationInterpolationCalculatorTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test

   /**
    * Numerically differentiate, then use OrientationInterpolationAngularVelocityCalculator, compare results.
    */
   public void testComputeAngularVelocity()
   {
      Random random = new Random(101L);

      ReferenceFrame world = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < 50000; i++)
      {
         FrameQuaternion startOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, world);
         FrameQuaternion endOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, world);

         double alpha = random.nextDouble();
         double alphaDot = random.nextDouble();
         double dt = 1e-6;
         double deltaAlpha = alphaDot * dt;
         double alphaNew = alpha + deltaAlpha;

         // compute angular velocity by numerical differentiation
         FrameQuaternion interpolatedOrientation0 = new FrameQuaternion(startOrientation.getReferenceFrame());
         interpolatedOrientation0.interpolate(startOrientation, endOrientation, alpha);

         FrameQuaternion interpolatedOrientationDt = new FrameQuaternion(startOrientation.getReferenceFrame());
         interpolatedOrientationDt.interpolate(startOrientation, endOrientation, alphaNew);

         Matrix3D interpolatedOrientationMatrixDot = new Matrix3D();
         RigidBodyTransform transformationAtDt = new RigidBodyTransform();
         transformationAtDt.getRotation().set(interpolatedOrientationDt);
         interpolatedOrientationMatrixDot.set(transformationAtDt.getRotation());
         RotationMatrix interpolatedOrientation0Matrix = new RotationMatrix();
         RigidBodyTransform transformationAt0 = new RigidBodyTransform();
         transformationAt0.getRotation().set(interpolatedOrientation0);
         interpolatedOrientation0Matrix.set(transformationAt0.getRotation());
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
         FrameVector3D angularVelocityFromCalculator = new FrameVector3D();
         orientationInterpolationCalculator.computeAngularVelocity(angularVelocityFromCalculator, startOrientation, endOrientation, alphaDot);

         double epsilon = 1e-5;
         assertEquals(angularVelocity.getX(), angularVelocityFromCalculator.getX(), epsilon);
         assertEquals(angularVelocity.getY(), angularVelocityFromCalculator.getY(), epsilon);
         assertEquals(angularVelocity.getZ(), angularVelocityFromCalculator.getZ(), epsilon);
      }
   }
}
