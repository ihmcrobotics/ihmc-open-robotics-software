package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class CoMTrajectorySegmentTest
{
   private static final int iters = 1000;

   @Test
   public void testCompute()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         CoMTrajectorySegment segment = new CoMTrajectorySegment();

         double startTime = RandomNumbers.nextDouble(random, 0.0, 3.0);
         double endTime = startTime + RandomNumbers.nextDouble(random, 0.0, 3.0);
         double omega = RandomNumbers.nextDouble(random, 0.1, 3.0);
         FramePoint3DReadOnly firstSegment = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3DReadOnly secondSegment = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3DReadOnly thirdSegment = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3DReadOnly fourthSegment = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3DReadOnly fifthSegment = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3DReadOnly sixthSegment = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         segment.setOmega(omega);
         segment.getTimeInterval().setInterval(startTime, endTime);
         segment.setFirstCoefficient(firstSegment);
         segment.setSecondCoefficient(secondSegment);
         segment.setThirdCoefficient(thirdSegment);
         segment.setFourthCoefficient(fourthSegment);
         segment.setFifthCoefficient(fifthSegment);
         segment.setSixthCoefficient(sixthSegment);

         assertFalse(segment.isDone());
         segment.compute(endTime + 0.1);
         assertTrue(segment.isDone());
         segment.compute(endTime - 0.1);
         assertFalse(segment.isDone());

         for (int i = 0; i < 10; i++)
         {
            double time = RandomNumbers.nextDouble(random, startTime, endTime);
            segment.compute(time);

            FramePoint3D comPosition = new FramePoint3D();
            FrameVector3D comVelocity = new FrameVector3D();
            FrameVector3D comAcceleration = new FrameVector3D();

            comPosition.scaleAdd(Math.exp(omega * time), firstSegment, comPosition);
            comPosition.scaleAdd(Math.exp(-omega * time), secondSegment, comPosition);
            comPosition.scaleAdd(time * time * time, thirdSegment, comPosition);
            comPosition.scaleAdd(time * time, fourthSegment, comPosition);
            comPosition.scaleAdd(time, fifthSegment, comPosition);
            comPosition.add(sixthSegment);

            comVelocity.scaleAdd(omega * Math.exp(omega * time), firstSegment, comVelocity);
            comVelocity.scaleAdd(-omega * Math.exp(-omega * time), secondSegment, comVelocity);
            comVelocity.scaleAdd(3.0 * time * time, thirdSegment, comVelocity);
            comVelocity.scaleAdd(2.0 * time, fourthSegment, comVelocity);
            comVelocity.add(fifthSegment);

            comAcceleration.scaleAdd(omega * omega * Math.exp(omega * time), firstSegment, comAcceleration);
            comAcceleration.scaleAdd(omega * omega * Math.exp(-omega * time), secondSegment, comAcceleration);
            comAcceleration.scaleAdd(6.0 * time, thirdSegment, comAcceleration);
            comAcceleration.scaleAdd(2.0, fourthSegment, comAcceleration);

            double epsilon = 1e-7;
            EuclidFrameTestTools.assertGeometricallyEquals(comPosition, segment.getPosition(), epsilon);
            EuclidFrameTestTools.assertGeometricallyEquals(comVelocity, segment.getVelocity(), epsilon);
            EuclidFrameTestTools.assertGeometricallyEquals(comAcceleration, segment.getAcceleration(), epsilon);

            FramePoint3D dcmPosition = new FramePoint3D();
            FrameVector3D dcmVelocity = new FrameVector3D();
            dcmPosition.scaleAdd(1.0 / omega, comVelocity, comPosition);
            dcmVelocity.scaleAdd(1.0 / omega, comAcceleration, comVelocity);

            EuclidFrameTestTools.assertGeometricallyEquals(dcmPosition, segment.getDCMPosition(), 1e-7);

            assertFalse(segment.isDone());
         }
      }
   }

   @Test
   public void testSet()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         CoMTrajectorySegment originalSegment = getRandomSegment(random);
         CoMTrajectorySegment segmentToSet = getRandomSegment(random);
         segmentToSet.set(originalSegment);

         for (int i = 0; i < 10; i++)
         {
            double time = RandomNumbers.nextDouble(random, originalSegment.getTimeInterval().getStartTime(), originalSegment.getTimeInterval().getEndTime());
            originalSegment.compute(time);
            segmentToSet.compute(time);

            EuclidFrameTestTools.assertGeometricallyEquals(originalSegment.getPosition(), segmentToSet.getPosition(), 1e-8);
            EuclidFrameTestTools.assertGeometricallyEquals(originalSegment.getVelocity(), segmentToSet.getVelocity(), 1e-8);
            EuclidFrameTestTools.assertGeometricallyEquals(originalSegment.getAcceleration(), segmentToSet.getAcceleration(), 1e-8);
            EuclidFrameTestTools.assertGeometricallyEquals(originalSegment.getDCMPosition(), segmentToSet.getDCMPosition(), 1e-8);
         }
      }
   }

   @Test
   public void testCropBeginningOfSegment()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         CoMTrajectorySegment originalSegment = new CoMTrajectorySegment();
         originalSegment.setOmega(RandomNumbers.nextDouble(random, 0.1, 3.0));
         double duration = RandomNumbers.nextDouble(random, 0.0, 3.0);
         originalSegment.getTimeInterval().setInterval(0.0, duration);
         originalSegment.setFirstCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
         originalSegment.setSecondCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
         originalSegment.setThirdCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
         originalSegment.setFourthCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
         originalSegment.setFifthCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
         originalSegment.setSixthCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));

         double timeToCrop = RandomNumbers.nextDouble(random, 0.0, duration);

         CoMTrajectorySegment segmentToCrop = getRandomSegment(random);
         segmentToCrop.set(originalSegment);
         segmentToCrop.shiftStartOfSegment(timeToCrop);

         for (double time = timeToCrop; time <= duration; time += 1e-3)
         {
            double timeInCropped = time - timeToCrop;
            originalSegment.compute(time);
            segmentToCrop.compute(timeInCropped);

            String failureMessage = "Failed at time " + time;
            EuclidFrameTestTools.assertGeometricallyEquals(failureMessage, originalSegment.getPosition(), segmentToCrop.getPosition(), 1e-8);
            EuclidFrameTestTools.assertGeometricallyEquals(failureMessage, originalSegment.getVelocity(), segmentToCrop.getVelocity(), 1e-8);
            EuclidFrameTestTools.assertGeometricallyEquals(failureMessage, originalSegment.getAcceleration(), segmentToCrop.getAcceleration(), 1e-8);
            EuclidFrameTestTools.assertGeometricallyEquals(failureMessage, originalSegment.getDCMPosition(), segmentToCrop.getDCMPosition(), 1e-8);
         }
      }
   }

   private static CoMTrajectorySegment getRandomSegment(Random random)
   {
      CoMTrajectorySegment segment = new CoMTrajectorySegment();
      segment.setOmega(RandomNumbers.nextDouble(random, 0.1, 3.0));
      double startTime = RandomNumbers.nextDouble(random, 0.0, 3.0);
      double endTime = startTime + RandomNumbers.nextDouble(random, 0.0, 3.0);
      segment.getTimeInterval().setInterval(startTime, endTime);
      segment.setFirstCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
      segment.setSecondCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
      segment.setThirdCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
      segment.setFourthCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
      segment.setFifthCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));
      segment.setSixthCoefficient(EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame()));

      return segment;
   }
}
