package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;

import java.util.Random;

public class CenterOfMassDynamicsToolsTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double epsilon = 1e-7;
   private static final int iters = 1000;

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
         CenterOfMassDynamicsTools.computeDesiredDCMPosition(omega, time, startDCM, startVRP, dcmToTest);

         FramePoint3D dcmExpected = new FramePoint3D();
         dcmExpected.sub(startDCM, startVRP);
         dcmExpected.scale(Math.exp(omega * time));
         dcmExpected.add(startVRP);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(dcmExpected, dcmToTest, epsilon);
      }
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
         time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         duration = RandomNumbers.nextDouble(random, 0.0, 1.0);

         FramePoint3D dcmToTest = new FramePoint3D();
         CenterOfMassDynamicsTools.computeDesiredDCMPositionForwardTime(omega, time, duration, startDCM, startVRP, endVRP, dcmToTest);

         double alpha = 1.0 - time / duration - 1.0 / (omega * duration) - Math.exp(omega * time) * (1.0 - 1.0 / (omega * duration));
         double beta = time / duration + 1.0 / (omega * duration) - Math.exp(omega * time) / (omega * duration);
         double gamma = Math.exp(omega * time);

         FramePoint3D dcmExpected = new FramePoint3D();
         dcmExpected.scaleAdd(alpha, startVRP, dcmExpected);
         dcmExpected.scaleAdd(beta, endVRP, dcmExpected);
         dcmExpected.scaleAdd(gamma, startDCM, dcmExpected);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(dcmExpected, dcmToTest, epsilon);
      }
   }

}
