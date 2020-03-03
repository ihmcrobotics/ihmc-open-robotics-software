package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class CoMTrajectoryPlannerToolsTest
{
   private static final double epsilon = 1e-4;


   @Test
   public void testTrajectoryConstruction()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1000; iter++)
      {
         double omega = RandomNumbers.nextDouble(random, 0.3, 2.0);
         double time = RandomNumbers.nextDouble(random, 0.0, 1.5);

         FramePoint3D c0 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c1 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c2 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c3 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c4 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c5 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());

         FramePoint3D desiredCoMPosition = new FramePoint3D();
         FramePoint3D desiredDCMPosition = new FramePoint3D();
         FramePoint3D desiredDCMPosition2 = new FramePoint3D();
         FramePoint3D desiredVRPPosition = new FramePoint3D();
         FramePoint3D desiredVRPPosition2 = new FramePoint3D();
         FrameVector3D desiredCoMVelocity = new FrameVector3D();
         FrameVector3D desiredCoMAcceleration = new FrameVector3D();
         FrameVector3D desiredDCMVelocity= new FrameVector3D();

         CoMTrajectoryPlannerTools.constructDesiredCoMPosition(desiredCoMPosition, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredCoMVelocity(desiredCoMVelocity, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredCoMAcceleration(desiredCoMAcceleration, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredVRPPosition(desiredVRPPosition2, c0, c1, c2, c3, c4, c5, time, omega);
         CoMTrajectoryPlannerTools.constructDesiredDCMPosition(desiredDCMPosition, c0, c1, c2, c3, c4, c5, time, omega);

         FramePoint3D desiredCoMPositionExpected = new FramePoint3D();
         FrameVector3D desiredCoMVelocityExpected = new FrameVector3D();
         FrameVector3D desiredCoMAccelerationExpected = new FrameVector3D();
         FramePoint3D desiredDCMPositionExpected = new FramePoint3D();
         FramePoint3D desiredDCMPositionExpected2 = new FramePoint3D();
         FrameVector3D desiredDCMVelocityExpected = new FrameVector3D();
         FramePoint3D desiredVRPPositionExpected = new FramePoint3D();
         FramePoint3D desiredVRPPositionExpected2 = new FramePoint3D();

         FramePoint3D temp = new FramePoint3D();

         // com position
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionFirstCoefficientTimeFunction(omega, time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionSecondCoefficientTimeFunction(omega, time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionThirdCoefficientTimeFunction(time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionFourthCoefficientTimeFunction(time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionFifthCoefficientTimeFunction(time));

         desiredCoMPositionExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getCoMPositionSixthCoefficientTimeFunction());

         desiredCoMPositionExpected.add(temp);

         // com velocity
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityFirstCoefficientTimeFunction(omega, time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocitySecondCoefficientTimeFunction(omega, time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityThirdCoefficientTimeFunction(time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityFourthCoefficientTimeFunction(time));

         desiredCoMVelocityExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocityFifthCoefficientTimeFunction());

         desiredCoMVelocityExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getCoMVelocitySixthCoefficientTimeFunction());

         desiredCoMVelocityExpected.add(temp);

         // com acceleration
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationFirstCoefficientTimeFunction(omega, time));

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationSecondCoefficientTimeFunction(omega, time));

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationThirdCoefficientTimeFunction(time));

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationFourthCoefficientTimeFunction());

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationFifthCoefficientTimeFunction());

         desiredCoMAccelerationExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getCoMAccelerationSixthCoefficientTimeFunction());

         desiredCoMAccelerationExpected.add(temp);

         // dcm position
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionFirstCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionSecondCoefficientTimeFunction());

         desiredDCMPositionExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionThirdCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionFourthCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionFifthCoefficientTimeFunction(omega, time));

         desiredDCMPositionExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getDCMPositionSixthCoefficientTimeFunction());

         desiredDCMPositionExpected.add(temp);

         // vrp position
         temp.set(c0);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionFirstCoefficientTimeFunction());

         desiredVRPPositionExpected.add(temp);

         temp.set(c1);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionSecondCoefficientTimeFunction());

         desiredVRPPositionExpected.add(temp);

         temp.set(c2);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionThirdCoefficientTimeFunction(omega, time));

         desiredVRPPositionExpected.add(temp);

         temp.set(c3);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionFourthCoefficientTimeFunction(omega, time));

         desiredVRPPositionExpected.add(temp);

         temp.set(c4);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionFifthCoefficientTimeFunction(time));

         desiredVRPPositionExpected.add(temp);

         temp.set(c5);
         temp.scale(CoMTrajectoryPlannerTools.getVRPPositionSixthCoefficientTimeFunction());

         desiredVRPPositionExpected.add(temp);

         CapturePointTools.computeCapturePointPosition(desiredCoMPositionExpected, desiredCoMVelocityExpected, omega, desiredDCMPositionExpected2);
         CapturePointTools.computeCapturePointPosition(desiredCoMPosition, desiredCoMVelocity, omega, desiredDCMPosition2);
         CapturePointTools.computeCapturePointVelocity(desiredCoMVelocity, desiredCoMAcceleration, omega, desiredDCMVelocity);
         CapturePointTools.computeCapturePointVelocity(desiredCoMVelocityExpected, desiredCoMAccelerationExpected, omega, desiredDCMVelocityExpected);
         CapturePointTools.computeCentroidalMomentumPivot(desiredDCMPosition, desiredDCMVelocity, omega, desiredVRPPosition);
         CapturePointTools.computeCentroidalMomentumPivot(desiredDCMPositionExpected, desiredDCMVelocityExpected, omega, desiredVRPPositionExpected2);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredCoMPositionExpected, desiredCoMPosition, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredCoMVelocityExpected, desiredCoMVelocity, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredCoMAccelerationExpected, desiredCoMAcceleration, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCMPositionExpected, desiredDCMPosition, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCMPositionExpected, desiredDCMPosition2, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCMPositionExpected2, desiredDCMPosition, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocityExpected, desiredDCMVelocity, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredVRPPositionExpected2, desiredVRPPosition, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredVRPPositionExpected2, desiredVRPPositionExpected, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredVRPPositionExpected2, desiredVRPPosition2, epsilon);
      }
   }
}
