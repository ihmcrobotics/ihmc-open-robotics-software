package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.mecano.tools.MultiBodySystemStateIntegrator;
import us.ihmc.robotics.math.QuaternionCalculus;

public class KSTToolsTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testComputeAngularVelocity()
   {
      Random random = new Random(7985395);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      QuaternionCalculus calculus = new QuaternionCalculus();

      for (int i = 0; i < 1000; i++)
      {
         double dt = random.nextDouble();
         FrameQuaternion previousOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameQuaternion currentOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D actual = new FrameVector3D();
         KSTTools.computeAngularVelocity(dt, previousOrientation, currentOrientation, actual);

         Vector4DBasics qDot = new Vector4D();
         calculus.computeQDotByFiniteDifferenceCentral(previousOrientation, currentOrientation, 0.5 * dt, qDot);
         Vector3DBasics expected = new Vector3D();
         calculus.computeAngularVelocityInBodyFixedFrame(currentOrientation, qDot, expected);

         EuclidCoreTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testIntegrateAngularVelocity()
   {
      Random random = new Random(54365);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < 1000; i++)
      {
         double dt = random.nextDouble();
         FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D angularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);

         FrameQuaternion actual = new FrameQuaternion();
         KSTTools.integrateAngularVelocity(dt, initialOrientation, angularVelocity, true, actual);

         Quaternion expected = new Quaternion();
         new MultiBodySystemStateIntegrator(dt).integrate(angularVelocity, initialOrientation, expected);

         EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expected, actual, EPSILON);
      }
   }
}
