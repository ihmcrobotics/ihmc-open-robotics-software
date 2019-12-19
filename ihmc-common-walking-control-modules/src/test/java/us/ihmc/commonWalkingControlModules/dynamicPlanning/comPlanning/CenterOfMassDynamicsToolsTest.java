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

public class CenterOfMassDynamicsToolsTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double integrationDt = 1e-7;

   private static final double epsilon = 1e-7;
   private static final int iters = 100;

   @Test
   public void testConstantVRPFunction()
   {
      Random random = new Random(1738L);
      double omega = 3.0;

      FramePoint3DReadOnly startDCM = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
      FramePoint3D startVRP = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
      double time = RandomNumbers.nextDouble(random, 1.0);

      FramePoint3D forwardDCM = new FramePoint3D();
      FramePoint3D backwardDCM = new FramePoint3D();
      CenterOfMassDynamicsTools.computeDesiredDCMPosition(omega, time, startDCM, startVRP, forwardDCM);
      CenterOfMassDynamicsTools.computeDesiredDCMPosition(omega, -time, forwardDCM, startVRP, backwardDCM);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startDCM, backwardDCM, epsilon);

      time = 0.0;
      CenterOfMassDynamicsTools.computeDesiredDCMPosition(omega, time, startDCM, startVRP, forwardDCM);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startDCM, forwardDCM, epsilon);

      time = RandomNumbers.nextDouble(random, 1.0);
      startVRP.set(startDCM);
      CenterOfMassDynamicsTools.computeDesiredDCMPosition(omega, time, startDCM, startVRP, forwardDCM);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startDCM, forwardDCM, epsilon);

      for (int i = 0; i < iters; i++)
      {
         startDCM = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
         startVRP = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
         time = RandomNumbers.nextDouble(random, 1.0);

         FramePoint3D dcmToTest = new FramePoint3D();
         FramePoint3D backwardDcmToTest = new FramePoint3D();
         CenterOfMassDynamicsTools.computeDesiredDCMPosition(omega, time, startDCM, startVRP, dcmToTest);
         CenterOfMassDynamicsTools.computeDesiredDCMPosition(omega, -time, dcmToTest, startVRP, backwardDcmToTest);

         FramePoint3D dcmExpected = new FramePoint3D();
         dcmExpected.sub(startDCM, startVRP);
         dcmExpected.scale(Math.exp(omega * time));
         dcmExpected.add(startVRP);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(dcmExpected, dcmToTest, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startDCM, backwardDcmToTest, epsilon);

         if (time >= 0.0)
         {
            FramePoint3DReadOnly integratedDCM = integrateDCMForwardInTimeWithConstantVRP(time, omega, startDCM, startVRP);
            EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(integratedDCM, dcmToTest, 1e-3);
         }
         else
         { // FIXME
//            FramePoint3DReadOnly integratedDCM = integrateDCMBackwardInTimeWithConstantVRP(-time, omega, startDCM, startVRP);
//            EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(integratedDCM, dcmToTest, 1e-3);
         }
      }
   }


   private static FramePoint3DReadOnly integrateDCMForwardInTimeWithConstantVRP(double integrationDuration, double omega, FramePoint3DReadOnly startDCM,
                                                                                FramePoint3DReadOnly startVRP)
   {
      FramePoint3D finalDCM = new FramePoint3D(startDCM);
      FrameVector3D dcmVelocity = new FrameVector3D();
      for (double time = 0.0; time <= integrationDuration; time += integrationDt)
      {
         dcmVelocity.sub(finalDCM, startVRP);
         dcmVelocity.scale(omega);

         finalDCM.scaleAdd(integrationDt, dcmVelocity, finalDCM);
      }

      return finalDCM;
   }

   private static FramePoint3DReadOnly integrateDCMBackwardInTimeWithConstantVRP(double integrationDuration, double omega, FramePoint3DReadOnly finalDCM,
                                                                                 FramePoint3DReadOnly vrp)
   {
      FramePoint3D startDCM = new FramePoint3D(finalDCM);
      FrameVector3D dcmVelocity = new FrameVector3D();
      for (double time = 0.0; time <= integrationDuration; time += integrationDt)
      {
         dcmVelocity.sub(startDCM, vrp);
         dcmVelocity.scale(-omega);

         startDCM.scaleAdd(integrationDt, dcmVelocity, finalDCM);
      }

      return startDCM;
   }


   @Test
   public void testLinearVRPFunction()
   {
      Random random = new Random(1738L);
      double omega = 3.0;

      FramePoint3DReadOnly startDCM = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
      FramePoint3D startVRP = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
      FramePoint3D endVRP = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
      double duration = RandomNumbers.nextDouble(random, 0.0, 1.0);
      double time = RandomNumbers.nextDouble(random, 0.0, duration);

      FramePoint3D vrpAtTime = new FramePoint3D();
      vrpAtTime.interpolate(startVRP, endVRP, time / duration);

      FramePoint3D forwardDCM = new FramePoint3D();
      FramePoint3D backwardDCM = new FramePoint3D();
      CenterOfMassDynamicsTools.computeDesiredDCMPositionForwardTime(omega, time, duration, startDCM, startVRP, endVRP, forwardDCM);
      CenterOfMassDynamicsTools.computeDesiredDCMPositionBackwardTime(omega, time, duration, forwardDCM, startVRP, endVRP, backwardDCM);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startDCM, backwardDCM, epsilon);

      CenterOfMassDynamicsTools.computeDesiredDCMPositionForwardTime(omega, duration, duration, startDCM, startVRP, endVRP, forwardDCM);
      CenterOfMassDynamicsTools.computeDesiredDCMPositionBackwardTime(omega, duration, duration, forwardDCM, startVRP, endVRP, backwardDCM);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startDCM, backwardDCM, epsilon);

      for (int i = 0; i < iters; i++)
      {
         startDCM = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
         startVRP = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
         endVRP = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 10.0);
         duration = RandomNumbers.nextDouble(random, 0.0, 1.0);
         time = RandomNumbers.nextDouble(random, 0.0, duration);

         FramePoint3D dcmToTest = new FramePoint3D();
         FramePoint3D backwardDcmToTest = new FramePoint3D();
         CenterOfMassDynamicsTools.computeDesiredDCMPositionForwardTime(omega, time, duration, startDCM, startVRP, endVRP, dcmToTest);
         CenterOfMassDynamicsTools.computeDesiredDCMPositionBackwardTime(omega, time, duration, dcmToTest, startVRP, endVRP, backwardDcmToTest);

         double alpha = 1.0 - time / duration - 1.0 / (omega * duration) - Math.exp(omega * time) * (1.0 - 1.0 / (omega * duration));
         double beta = time / duration + 1.0 / (omega * duration) - Math.exp(omega * time) / (omega * duration);
         double gamma = Math.exp(omega * time);

         FramePoint3D dcmExpected = new FramePoint3D();
         dcmExpected.scaleAdd(alpha, startVRP, dcmExpected);
         dcmExpected.scaleAdd(beta, endVRP, dcmExpected);
         dcmExpected.scaleAdd(gamma, startDCM, dcmExpected);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(dcmExpected, dcmToTest, epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(startDCM, backwardDcmToTest, epsilon);

         FramePoint3DReadOnly integratedDCM = integrateDCMForwardInTimeWithLinearVRP(time, duration, omega, startDCM, startVRP, endVRP);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(integratedDCM, dcmToTest, 1e-3);

         FramePoint3DReadOnly backwardIntegratedDCM = integrateDCMBackwardInTimeWithLinearVRP(time, duration, omega, integratedDCM, startVRP, endVRP);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(backwardIntegratedDCM, backwardDcmToTest, 1e-3);
      }
   }

   private static FramePoint3DReadOnly integrateDCMForwardInTimeWithLinearVRP(double integrationDuration, double totalDuration, double omega,
                                                                              FramePoint3DReadOnly startDCM, FramePoint3DReadOnly startVRP,
                                                                              FramePoint3DReadOnly endVRP)
   {
      FramePoint3D finalDCM = new FramePoint3D(startDCM);
      FramePoint3D desiredVRP = new FramePoint3D();
      FrameVector3D dcmVelocity = new FrameVector3D();
      for (double time = 0.0; time <= integrationDuration; time += integrationDt)
      {
         desiredVRP.interpolate(startVRP, endVRP, time / totalDuration);

         dcmVelocity.sub(finalDCM, desiredVRP);
         dcmVelocity.scale(omega);

         finalDCM.scaleAdd(integrationDt, dcmVelocity, finalDCM);
      }

      return finalDCM;
   }

   private static FramePoint3DReadOnly integrateDCMBackwardInTimeWithLinearVRP(double integrationDuration, double totalDuration, double omega,
                                                                              FramePoint3DReadOnly finalDCM, FramePoint3DReadOnly startVRP,
                                                                              FramePoint3DReadOnly endVRP)
   {
      FramePoint3D startDCM = new FramePoint3D(finalDCM);
      FramePoint3D desiredVRP = new FramePoint3D();
      FrameVector3D dcmVelocity = new FrameVector3D();
      for (double time = integrationDuration; time >= 0.0; time -= integrationDt)
      {
         desiredVRP.interpolate(startVRP, endVRP, time / totalDuration);

         dcmVelocity.sub(startDCM, desiredVRP);
         dcmVelocity.scale(omega);

         startDCM.scaleAdd(-integrationDt, dcmVelocity, startDCM);
      }

      return startDCM;
   }
}
