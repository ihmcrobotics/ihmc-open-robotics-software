package us.ihmc.commonWalkingControlModules.capturePoint;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.Assert;

import java.util.Random;

public class CapturePointToolsTest
{
   private static final int iters = 1000;
   private static final double omega = 3.0;

   @Test
   public void testComputeCapturePointPosition()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FramePoint3DReadOnly comPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FrameVector3DReadOnly comVelocity = nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), 10.0);

         FramePoint2DReadOnly comPosition2d = new FramePoint2D(comPosition);
         FrameVector2DReadOnly comVelocity2d = new FrameVector2D(comVelocity);

         FramePoint3D icpPosition = new FramePoint3D();
         FramePoint2D icpPosition2d = new FramePoint2D();
         FramePoint3D icpPositionExpected = new FramePoint3D();
         FramePoint2D icpPosition2dExpected = new FramePoint2D();

         CapturePointTools.computeCapturePointPosition(comPosition2d, comVelocity2d, omega, icpPosition2d);
         CapturePointTools.computeCapturePointPosition(comPosition, comVelocity, omega, icpPosition);
         icpPositionExpected.scaleAdd(1.0 / omega, comVelocity, comPosition);
         icpPosition2dExpected.scaleAdd(1.0 / omega, comVelocity2d, comPosition2d);

         EuclidFrameTestTools.assertGeometricallyEquals(icpPositionExpected, icpPosition, 1e-5);
         EuclidFrameTestTools.assertGeometricallyEquals(icpPosition2dExpected, icpPosition2d, 1e-5);
      }
   }

   @Test
   public void testComputeCapturePointVelocity()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FrameVector3DReadOnly comVelocity = nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FrameVector3DReadOnly comAcceleration = nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), 10.0);

         FrameVector2DReadOnly comVelocity2d = new FrameVector2D(comVelocity);
         FrameVector2DReadOnly comAcceleration2d = new FrameVector2D(comAcceleration);

         FrameVector3D icpVelocity = new FrameVector3D();
         FrameVector2D icpVelocity2d = new FrameVector2D();
         FrameVector3D icpVelocityExpected = new FrameVector3D();
         FrameVector2D icpVelocity2dExpected = new FrameVector2D();

         CapturePointTools.computeCapturePointVelocity(comVelocity2d, comAcceleration2d, omega, icpVelocity2d);
         CapturePointTools.computeCapturePointVelocity(comVelocity, comAcceleration, omega, icpVelocity);
         icpVelocityExpected.scaleAdd(1.0 / omega, comAcceleration, comVelocity);
         icpVelocity2dExpected.scaleAdd(1.0 / omega, comAcceleration2d, comVelocity2d);

         EuclidFrameTestTools.assertGeometricallyEquals(icpVelocityExpected, icpVelocity, 1e-5);
         EuclidFrameTestTools.assertGeometricallyEquals(icpVelocity2dExpected, icpVelocity2d, 1e-5);

         FramePoint3D comPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FramePoint3D vrpPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);

         CapturePointTools.computeCapturePointVelocity(comPosition, vrpPosition, omega, icpVelocity);
         icpVelocityExpected.sub(comPosition, vrpPosition);
         icpVelocityExpected.scale(omega);

         EuclidFrameTestTools.assertGeometricallyEquals(icpVelocityExpected, icpVelocity, 1e-5);
      }
   }

   @Test
   public void testCentroidalMomentumPivot()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FramePoint3DReadOnly icpPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FrameVector3DReadOnly icpVelocity = nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), 10.0);

         FramePoint2DReadOnly icpPosition2d = new FramePoint2D(icpPosition);
         FrameVector2DReadOnly icpVelocity2d = new FrameVector2D(icpVelocity);

         FramePoint3D cmpPosition = new FramePoint3D();
         FramePoint2D cmpPosition2d = new FramePoint2D();
         FramePoint3D cmpPositionExpected = new FramePoint3D();
         FramePoint2D cmpPosition2dExpected = new FramePoint2D();

         CapturePointTools.computeCentroidalMomentumPivot(icpPosition2d, icpVelocity2d, omega, cmpPosition2d);
         CapturePointTools.computeCentroidalMomentumPivot(icpPosition, icpVelocity, omega, cmpPosition);
         cmpPositionExpected.scaleAdd(-1.0 / omega, icpVelocity, icpPosition);
         cmpPosition2dExpected.scaleAdd(-1.0 / omega, icpVelocity2d, icpPosition2d);

         EuclidFrameTestTools.assertGeometricallyEquals(cmpPositionExpected, cmpPosition, 1e-5);
         EuclidFrameTestTools.assertGeometricallyEquals(cmpPosition2dExpected, cmpPosition2d, 1e-5);
      }
   }

   @Test
   public void testComputeDesiredCapturePointPosition()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         double duration = RandomNumbers.nextDouble(random, 0.0, 2.0);

         FramePoint3DReadOnly initialIcpPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FramePoint3DReadOnly cmpPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);

         FramePoint2DReadOnly initialIcpPosition2d = new FramePoint2D(initialIcpPosition);
         FramePoint2DReadOnly cmpPosition2d = new FramePoint2D(cmpPosition);

         FramePoint3D nextICPPosition = new FramePoint3D();
         FramePoint2D nextICPPosition2D = new FramePoint2D();
         FramePoint3D nextICPPositionExpected = new FramePoint3D();
         FramePoint2D nextICPPosition2dExpected = new FramePoint2D();

         CapturePointTools.computeDesiredCapturePointPosition(omega, duration, initialIcpPosition, cmpPosition, nextICPPosition);
         CapturePointTools.computeDesiredCapturePointPosition(omega, duration, initialIcpPosition2d, cmpPosition2d, nextICPPosition2D);

         nextICPPositionExpected.sub(initialIcpPosition, cmpPosition);
         nextICPPosition2dExpected.sub(initialIcpPosition2d, cmpPosition2d);

         nextICPPositionExpected.scale(Math.exp(omega * duration));
         nextICPPosition2dExpected.scale(Math.exp(omega * duration));

         nextICPPositionExpected.add(cmpPosition);
         nextICPPosition2dExpected.add(cmpPosition2d);

         EuclidFrameTestTools.assertGeometricallyEquals(nextICPPositionExpected, nextICPPosition, 1e-5);
         EuclidFrameTestTools.assertGeometricallyEquals(nextICPPosition2dExpected, nextICPPosition2D, 1e-5);
      }

      FramePoint3DReadOnly initialIcpPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);
      FramePoint3DReadOnly cmpPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);

      double dt = 1e-5;
      double time = dt;
      double duration = 1.0;
      FramePoint3D expectedIcpPosition = new FramePoint3D(initialIcpPosition);

      for (; time <= duration; time += dt)
      {
         FramePoint3D icpPosition = new FramePoint3D();
         CapturePointTools.computeDesiredCapturePointPosition(omega, time, initialIcpPosition, cmpPosition, icpPosition);

         FrameVector3D icpVelocity = new FrameVector3D();
         CapturePointTools.computeCapturePointVelocity(expectedIcpPosition, cmpPosition, omega, icpVelocity);

         expectedIcpPosition.scaleAdd(dt, icpVelocity, expectedIcpPosition);

         EuclidFrameTestTools.assertGeometricallyEquals("Failed at time " + time, expectedIcpPosition, icpPosition, 1e-2);
      }
   }

   @Test
   public void testComputeDesiredCapturePointPositionWithLinearCMP()
   {
      Random random = new Random(1738L);
//
//      for (int iter = 0; iter < iters; iter++)
//      {
//         double duration = RandomNumbers.nextDouble(random, 0.0, 2.0);
//
//         FramePoint3DReadOnly initialIcpPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);
//         FramePoint3DReadOnly cmpPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);
//
//         FramePoint2DReadOnly initialIcpPosition2d = new FramePoint2D(initialIcpPosition);
//         FramePoint2DReadOnly cmpPosition2d = new FramePoint2D(cmpPosition);
//
//         FramePoint3D nextICPPosition = new FramePoint3D();
//         FramePoint2D nextICPPosition2D = new FramePoint2D();
//         FramePoint3D nextICPPositionExpected = new FramePoint3D();
//         FramePoint2D nextICPPosition2dExpected = new FramePoint2D();
//
//         CapturePointTools.computeDesiredCapturePointPosition(omega, duration, initialIcpPosition, cmpPosition, nextICPPosition);
//         CapturePointTools.computeDesiredCapturePointPosition(omega, duration, initialIcpPosition2d, cmpPosition2d, nextICPPosition2D);
//
//         nextICPPositionExpected.sub(initialIcpPosition, cmpPosition);
//         nextICPPosition2dExpected.sub(initialIcpPosition2d, cmpPosition2d);
//
//         nextICPPositionExpected.scale(Math.exp(omega * duration));
//         nextICPPosition2dExpected.scale(Math.exp(omega * duration));
//
//         nextICPPositionExpected.add(cmpPosition);
//         nextICPPosition2dExpected.add(cmpPosition2d);
//
//         EuclidFrameTestTools.assertGeometricallyEquals(nextICPPositionExpected, nextICPPosition, 1e-5);
//         EuclidFrameTestTools.assertGeometricallyEquals(nextICPPosition2dExpected, nextICPPosition2D, 1e-5);
//      }

      FramePoint2DReadOnly initialIcpPosition = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0);
      FramePoint2DReadOnly startCmpPosition = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0);
      FramePoint2DReadOnly endCmpPosition = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0);

      double dt = 1e-5;
      double time = dt;
      double duration = 1.0;
      FramePoint2D expectedIcpPosition = new FramePoint2D(initialIcpPosition);
      FramePoint2D desiredCMP = new FramePoint2D();

      for (; time <= duration; time += dt)
      {
         FramePoint2D icpPosition = new FramePoint2D();
         CapturePointTools.computeDesiredCapturePointPosition(omega, time, duration, initialIcpPosition, startCmpPosition, endCmpPosition, icpPosition);

         FrameVector2D icpVelocity = new FrameVector2D();
         desiredCMP.interpolate(startCmpPosition, endCmpPosition, (time - dt) / duration);

         CapturePointTools.computeCapturePointVelocity(expectedIcpPosition, desiredCMP, omega, icpVelocity);

         expectedIcpPosition.scaleAdd(dt, icpVelocity, expectedIcpPosition);

         EuclidFrameTestTools.assertGeometricallyEquals("Failed at time " + time, expectedIcpPosition, icpPosition, 1e-3);
      }
   }

   @Test
   public void testComputeTimeToReachCapturePointUsingConstantCMP()
   {
      Random random = new Random(1738L);

      for(int i = 0; i < iters; i++)
      {
         double swingTime = RandomNumbers.nextDouble(random, 0.01, 2.0);

         FramePoint2D initialCapturePoint = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame());
         FramePoint2D constantCMP = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame());

         FramePoint2D finalCapturePoint = new FramePoint2D();
         CapturePointTools.computeDesiredCapturePointPosition(omega, swingTime, initialCapturePoint, constantCMP, finalCapturePoint);

         double estimatedSwingTime = CapturePointTools.computeTimeToReachCapturePointUsingConstantCMP(omega, finalCapturePoint, initialCapturePoint, constantCMP);

         Assert.assertEquals(swingTime, estimatedSwingTime, 1.0e-6);
      }
   }



   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame referenceFrame, double max)
   {
      return new FrameVector3D(referenceFrame, EuclidCoreRandomTools.nextVector3D(random, -max, max));
   }
}
