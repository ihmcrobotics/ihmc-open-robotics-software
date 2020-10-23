package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FootRotationDetectorTest
{
   @Test
   public void testRotationDetectionMath()
   {
      Random random = new Random(429L);
      YoRegistry registry = new YoRegistry(FeetManager.class.getSimpleName());
      double dt = 0.001;
      RobotSide side = RobotSide.LEFT;
      Twist soleTwist = new Twist();
      MovingReferenceFrame soleFrame = new TestSoleFrame(soleTwist);
      soleTwist.setToZero(soleFrame, ReferenceFrame.getWorldFrame(), soleFrame);
      FootRotationDetector footRotationDetector = new FootRotationDetector(side, soleFrame, dt, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

      // Test for planar measurement:
      for (int i = 0; i < 100; i++)
      {
         // create random cop location (zero linear velocity) and a rotational velocity
         double omegaNorm = random.nextDouble() + 10.0;
         FramePoint3D point = new FramePoint3D(soleFrame, EuclidCoreRandomTools.nextPoint2D(random));
         FrameVector3D omega = new FrameVector3D(soleFrame, EuclidCoreRandomTools.nextVector2DWithFixedLength(random, omegaNorm));

         // update the sole twist based on the cop location and the angular velocity
         soleTwist.setIncludingFrame(omega, new FrameVector3D(soleFrame), point);
         soleFrame.update();

         // try to estimate the line of rotation which should go through the cop and have the direction of omega
         footRotationDetector.reset();
         footRotationDetector.compute();

         FrameLine2DReadOnly lineEstimate = footRotationDetector.getLineOfRotation();
         FrameLine2D expectedLine = new FrameLine2D(soleFrame, new FramePoint2D(point), new FrameVector2D(omega));
         EuclidGeometryTestTools.assertLine2DGeometricallyEquals(expectedLine, lineEstimate, 1.0e-5);
      }

      // Test for non-planar measurement:
      for (int i = 0; i < 100; i++)
      {
         // create random cop location (zero linear velocity) and a rotational velocity
         double omegaNorm = random.nextDouble() + 10.0;
         FramePoint3D point = new FramePoint3D(soleFrame, EuclidCoreRandomTools.nextPoint3D(random));
         FrameVector3D omega = new FrameVector3D(soleFrame, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, omegaNorm));

         // update the sole twist based on the cop location and the angular velocity
         soleTwist.setIncludingFrame(omega, new FrameVector3D(soleFrame), point);
         soleFrame.update();

         // try to estimate the line of rotation which should go through the cop and have the direction of omega
         footRotationDetector.reset();
         footRotationDetector.compute();

         FrameLine2DReadOnly lineEstimate = footRotationDetector.getLineOfRotation();
         FrameLine2D expectedLine = new FrameLine2D(soleFrame, new FramePoint2D(point), new FrameVector2D(omega));
         EuclidGeometryTestTools.assertLine2DGeometricallyEquals(expectedLine, lineEstimate, 1.0e-5);
      }
   }

   private class TestSoleFrame extends MovingReferenceFrame
   {
      private final Twist twistRelativeToParent;

      public TestSoleFrame(Twist twistRelativeToParent)
      {
         super("TestFrame", ReferenceFrame.getWorldFrame());
         this.twistRelativeToParent = twistRelativeToParent;
      }

      @Override
      protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
      {
         twistRelativeToParentToPack.set(twistRelativeToParent);
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setToZero();
      }

   }
}
