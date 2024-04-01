package us.ihmc.commonWalkingControlModules.capturePoint;

import org.junit.jupiter.api.Test;
import org.ojalgo.random.RandomNumber;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.FrameEllipsoid3D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class ALIPToolsTest
{
   private static final int iters = 1000;

   @Test
   public void testComputeCenterOfMassPosition()
   {
      double omega = 3.0;
      double totalMass = 30.0;
      double gravity = 9.81;
      double desiredHeight = gravity / (omega * omega);

      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FramePoint2D terminalCoMPosition = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FrameVector2D terminalAngularMomentum = EuclidFrameRandomTools.nextFrameVector2D(random, ReferenceFrame.getWorldFrame(), -10.0, 10.0);
         FramePoint2D terminalPendulumBase = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 1.0);
         terminalPendulumBase.add(terminalCoMPosition);

         double timeToComputeBackwards = RandomNumbers.nextDouble(random, 0.001, 1.5);


         double relativeCoMPositionX = terminalCoMPosition.getX() - terminalPendulumBase.getX();
         double relativeCoMPositionY = terminalCoMPosition.getY() - terminalPendulumBase.getY();

         double wT = omega * timeToComputeBackwards;
         double coshWt = Math.cosh(wT);
         double sinhWt = Math.sinh(wT);
         double equivalentVelocityX = terminalAngularMomentum.getY() / (totalMass * desiredHeight);
         double equivalentVelocityY = -terminalAngularMomentum.getX() / (totalMass * desiredHeight);
         double computedInitialX = (relativeCoMPositionX * coshWt - equivalentVelocityX * sinhWt / omega) + terminalPendulumBase.getX();
         double computedInitialY = (relativeCoMPositionY * coshWt - equivalentVelocityY * sinhWt / omega) + terminalPendulumBase.getY();

         FramePoint2D initialCoM = new FramePoint2D();
         FramePoint2D computedCom = new FramePoint2D(ReferenceFrame.getWorldFrame(), computedInitialX, computedInitialY);

         ALIPTools.computeCenterOfMassPosition(-timeToComputeBackwards, omega, totalMass, gravity, terminalCoMPosition, terminalAngularMomentum, terminalPendulumBase, initialCoM);

         EuclidFrameTestTools.assertEquals(computedCom, initialCoM, 1e-6);
      }
   }

   @Test
   public void testComputeAngularMomentum()
   {
      double omega = 3.0;
      double totalMass = 30.0;
      double gravity = 9.81;
      double desiredHeight = gravity / (omega * omega);

      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FramePoint2D terminalCoMPosition = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FrameVector2D terminalAngularMomentum = EuclidFrameRandomTools.nextFrameVector2D(random, ReferenceFrame.getWorldFrame(), -10.0, 10.0);
         FramePoint2D terminalPendulumBase = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 1.0);
         terminalPendulumBase.add(terminalCoMPosition);

         double timeToComputeBackwards = RandomNumbers.nextDouble(random, 0.001, 1.5);

         double relativePositionX = terminalCoMPosition.getX() - terminalPendulumBase.getX();
         double relativePositionY = terminalCoMPosition.getY() - terminalPendulumBase.getY();

         double wT = omega * timeToComputeBackwards;
         double coshWt = Math.cosh(wT);
         double sinhWt = Math.sinh(wT);
         double computedInitialAMY = (-relativePositionX * sinhWt * totalMass * desiredHeight * omega + terminalAngularMomentum.getY() * coshWt);
         // FIXME check the signs because cross products. Then check the sines because. Then check the signs for fortune to come.
         double computedInitialAMX = (relativePositionY * sinhWt * totalMass * desiredHeight * omega + terminalAngularMomentum.getX() * coshWt);

         FrameVector2D initialAM = new FrameVector2D();
         FrameVector2D computedInitialAM = new FrameVector2D(ReferenceFrame.getWorldFrame(), computedInitialAMX, computedInitialAMY);

         ALIPTools.computeAngularMomentum(-timeToComputeBackwards, omega, totalMass, gravity, terminalCoMPosition, terminalAngularMomentum, terminalPendulumBase, initialAM);

         EuclidFrameTestTools.assertEquals(computedInitialAM, initialAM, 1e-6);
      }
   }

   @Test
   public void testComputeACP()
   {
      double omega = 3.0;
      double totalMass = 30.0;
      double gravity = 9.81;
      double desiredHeight = gravity / (omega * omega);

      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FramePoint2D comPosition = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FrameVector2D angularMomentum = EuclidFrameRandomTools.nextFrameVector2D(random, ReferenceFrame.getWorldFrame(), -10.0, 10.0);

         double equivalentVelocityX = angularMomentum.getY() / (totalMass * desiredHeight);
         double equivalentVelocityY = -angularMomentum.getX() / (totalMass * desiredHeight);

         FramePoint2D acpExpected = new FramePoint2D();
         acpExpected.setIncludingFrame(comPosition);
         acpExpected.addX(equivalentVelocityX / omega);
         acpExpected.addY(equivalentVelocityY / omega);

         FramePoint2D acp = new FramePoint2D();

         ALIPTools.computeACP(omega, totalMass, gravity, comPosition, angularMomentum, acp);

         EuclidFrameTestTools.assertEquals(acpExpected, acp, 1e-7);
      }

   }


   @Test
   public void testComparisonAgainstACPUsingALIPDynamics()
   {
      double omega = 3.0;
      double totalMass = 30.0;
      double gravity = 9.81;

      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         FramePoint2D comPosition = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FrameVector2D angularMomentum = EuclidFrameRandomTools.nextFrameVector2D(random, ReferenceFrame.getWorldFrame(), -10.0, 10.0);
         FramePoint2D pendulumBase = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 1.0);
         pendulumBase.add(comPosition);

         double timeForwards = RandomNumbers.nextDouble(random, 0.001, 1.5);

         FramePoint2D futureCoMPosition = new FramePoint2D();
         FrameVector2D futureAM = new FrameVector2D();
         ALIPTools.computeCenterOfMassPosition(timeForwards, omega, totalMass, gravity, comPosition, angularMomentum, pendulumBase, futureCoMPosition);
         ALIPTools.computeAngularMomentum(timeForwards, omega, totalMass, gravity, comPosition, angularMomentum, pendulumBase, futureAM);

         FramePoint2D acp = new FramePoint2D();
         ALIPTools.computeACP(omega, totalMass, gravity, comPosition, angularMomentum, acp);

         FramePoint2D futureACPExpected = new FramePoint2D();
         CapturePointTools.computeDesiredCapturePointPosition(omega, timeForwards, acp, pendulumBase, futureACPExpected);

         FramePoint2D futureACP = new FramePoint2D();
         ALIPTools.computeACP(omega, totalMass, gravity, futureCoMPosition, futureAM, futureACP);

         // we know that projecting the ALIP dynamics forward in time should result in the same ACP position as if we had projected the ACP dynamics forward in
         // time. Here, we're checking and asserting that.
         EuclidFrameTestTools.assertEquals(futureACPExpected, futureACP, 1e-6);
      }
   }
}
