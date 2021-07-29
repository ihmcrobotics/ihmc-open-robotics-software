package us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling;

import org.junit.jupiter.api.Test;
import org.ojalgo.random.RandomNumber;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;

import java.util.Random;

public class ContactSegmentHelperTest
{
   private static final int iters = 10000;
   private static final double epsilon = 1e-6;

   @Test
   public void testRandomCubicTrajectory()
   {
      Random random = new Random(1738L);
      ContactSegmentHelper helper = new ContactSegmentHelper();
      for (int i = 0; i < iters; i++)
      {
         Polynomial3D trajectory = new Polynomial3D(4);
         Polynomial3D trajectory2 = new Polynomial3D(4);

         FramePoint3D startPoint = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D endPoint = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D startVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D endVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         SettableContactStateProvider segmentToInterpolateFromStart = new SettableContactStateProvider();
         SettableContactStateProvider segmentToInterpolateFromEnd = new SettableContactStateProvider();

         segmentToInterpolateFromStart.setStartECMPPosition(startPoint);
         segmentToInterpolateFromStart.setEndECMPPosition(endPoint);
         segmentToInterpolateFromStart.setStartECMPVelocity(startVelocity);
         segmentToInterpolateFromStart.setEndECMPVelocity(endVelocity);

         segmentToInterpolateFromEnd.setStartECMPPosition(startPoint);
         segmentToInterpolateFromEnd.setEndECMPPosition(endPoint);
         segmentToInterpolateFromEnd.setStartECMPVelocity(startVelocity);
         segmentToInterpolateFromEnd.setEndECMPVelocity(endVelocity);

         double startTime = RandomNumbers.nextDouble(random, 10.0);
         double duration = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double endTime = startTime + duration;
         double timeFromStart = RandomNumbers.nextDouble(random, startTime, endTime);
         double timeFromEnd = RandomNumbers.nextDouble(random, startTime, endTime);

         double alphaFromStart = (timeFromStart - startTime) / duration;
         double alphaFromEnd = (timeFromEnd - startTime) / duration;

         trajectory.setCubic(startTime, endTime, startPoint, startVelocity, endPoint, endVelocity);
         trajectory2.setCubic(0.0, duration, startPoint, startVelocity, endPoint, endVelocity);

         helper.cubicInterpolateStartOfSegment(segmentToInterpolateFromStart, alphaFromStart);
         helper.cubicInterpolateEndOfSegment(segmentToInterpolateFromEnd, alphaFromEnd);

         trajectory.compute(timeFromStart);
         trajectory2.compute(timeFromStart - startTime);

         FramePoint3D alt = new FramePoint3D();

         interpolatePosition(alt, startPoint, startVelocity, endPoint, endVelocity, alphaFromStart);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(trajectory.getPosition(), trajectory2.getPosition(), epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectory.getVelocity(), trajectory2.getVelocity(), epsilon);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(alt, segmentToInterpolateFromStart.getECMPStartPosition(), epsilon);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(trajectory.getPosition(), segmentToInterpolateFromStart.getECMPStartPosition(), epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectory.getVelocity(), segmentToInterpolateFromStart.getECMPStartVelocity(), epsilon);

         trajectory.compute(timeFromEnd);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(trajectory.getPosition(), segmentToInterpolateFromEnd.getECMPEndPosition(), epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(trajectory.getVelocity(), segmentToInterpolateFromEnd.getECMPEndVelocity(), epsilon);
      }
   }

   @Test
   public void testRandomLinearTrajectory()
   {
      Random random = new Random(1738L);
      ContactSegmentHelper helper = new ContactSegmentHelper();
      for (int i = 0; i < iters; i++)
      {

         FramePoint3D startPoint = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D endPoint = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         SettableContactStateProvider segmentToInterpolateFromStart = new SettableContactStateProvider();
         SettableContactStateProvider segmentToInterpolateFromEnd = new SettableContactStateProvider();

         segmentToInterpolateFromStart.setStartECMPPosition(startPoint);
         segmentToInterpolateFromStart.setEndECMPPosition(endPoint);
         segmentToInterpolateFromStart.setLinearECMPVelocity();

         segmentToInterpolateFromEnd.setStartECMPPosition(startPoint);
         segmentToInterpolateFromEnd.setEndECMPPosition(endPoint);
         segmentToInterpolateFromEnd.setLinearECMPVelocity();

         double startTime = RandomNumbers.nextDouble(random, 10.0);
         double duration = RandomNumbers.nextDouble(random, 0.0, 10.0);
         double endTime = startTime + duration;
         double timeFromStart = RandomNumbers.nextDouble(random, startTime, endTime);
         double timeFromEnd = RandomNumbers.nextDouble(random, startTime, endTime);

         double alphaFromStart = (timeFromStart - startTime) / duration;
         double alphaFromEnd = (timeFromEnd - startTime) / duration;


         FrameVector3DBasics velocity = new FrameVector3D();
         velocity.sub(endPoint, startPoint);
         velocity.scale(1.0 / duration);

         FramePoint3D positionFromStart = new FramePoint3D();
         FramePoint3D positionFromEnd = new FramePoint3D();

         positionFromStart.interpolate(startPoint, endPoint, alphaFromStart);
         positionFromEnd.interpolate(startPoint, endPoint, alphaFromEnd);

         helper.cubicInterpolateStartOfSegment(segmentToInterpolateFromStart, alphaFromStart);
         helper.cubicInterpolateEndOfSegment(segmentToInterpolateFromEnd, alphaFromEnd);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(positionFromStart, segmentToInterpolateFromStart.getECMPStartPosition(), epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, segmentToInterpolateFromStart.getECMPStartVelocity(), epsilon);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(positionFromEnd, segmentToInterpolateFromEnd.getECMPEndPosition(), epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(velocity, segmentToInterpolateFromEnd.getECMPEndVelocity(), epsilon);
      }
   }

   private static void interpolatePosition(FramePoint3DBasics framePointToPack, FramePoint3DReadOnly startPosition,
                                           FrameVector3DReadOnly startVelocity, FramePoint3DReadOnly endPosition,
                                           FrameVector3DReadOnly endVelocity, double alpha)
   {
      FramePoint3D c2 = new FramePoint3D();
      c2.sub(endPosition, startPosition);
      c2.scale(3.0);
      c2.scaleAdd(-2.0, startVelocity, c2);
      c2.sub(endVelocity);

      FramePoint3D c3 = new FramePoint3D();
      c3.sub(startPosition, endPosition);
      c3.scale(2.0);
      c3.add(startVelocity);
      c3.add(endVelocity);

      framePointToPack.set(startPosition);
      framePointToPack.scaleAdd(alpha, startVelocity, framePointToPack);
      framePointToPack.scaleAdd(alpha * alpha, c2, framePointToPack);
      framePointToPack.scaleAdd(alpha * alpha * alpha, c3, framePointToPack);
   }
}
