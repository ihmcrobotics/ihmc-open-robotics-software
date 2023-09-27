package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.apache.batik.ext.awt.geom.Linear;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class CoMOptimalContinuityCalculatorTest
{
   private static final double epsilon = 1e-7;

   @Test
   public void testContinuity()
   {
      double omega = 3.0;
      YoDouble omegaProvider = new YoDouble("omega", new YoRegistry("test"));
      omegaProvider.set(omega);
      CoMOptimalContinuityCalculator calculator = new CoMOptimalContinuityCalculator(9.81, omegaProvider, new YoRegistry("test"));

      double height = 9.81 / MathTools.square(omega);

      FramePoint3D finalDesiredDCM = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.5, height);
      FramePoint3D initialCoM = new FramePoint3D();
      FramePoint3D midpoint = new FramePoint3D();
      FrameVector3D initialCoMVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.25, 0.25, 0.3);
      initialCoM.setZ(height);
      midpoint.interpolate(initialCoM, finalDesiredDCM, 0.5);

      calculator.setFinalICPToAchieve(finalDesiredDCM);
      calculator.setInitialCoMPosition(initialCoM);
      calculator.setInitialCoMVelocity(initialCoMVelocity);

      List<ContactStateProvider> contacts = new ArrayList<>();
      SettableContactStateProvider contact1 = new SettableContactStateProvider();
      SettableContactStateProvider contact2 = new SettableContactStateProvider();

      double firstDuration = 0.25;
      double secondDuration = 0.5;
      initialCoM.subZ(height);
      midpoint.subZ(height);
      finalDesiredDCM.subZ(height);

      contact1.setStartECMPPosition(initialCoM);
      contact1.setEndECMPPosition(midpoint);
      contact2.setStartECMPPosition(midpoint);
      contact2.setEndECMPPosition(finalDesiredDCM);
      contact1.getTimeInterval().setInterval(0.0, firstDuration);
      contact2.getTimeInterval().setInterval(firstDuration, firstDuration + secondDuration);
      contact1.setLinearECMPVelocity();
      contact2.setLinearECMPVelocity();

      initialCoM.addZ(height);
      midpoint.addZ(height);
      finalDesiredDCM.addZ(height);

      contacts.add(contact1);
      contacts.add(contact2);

      calculator.solve(contacts);

      LinearCoMTrajectoryHandler trajectory = calculator.getDebugTrajectory();

      FramePoint3D achievedInitialCoMPosition = new FramePoint3D();
      FrameVector3D achievedInitialCoMVelocity = new FrameVector3D();
      trajectory.computeCoMPosition(0, 0.0, achievedInitialCoMPosition);
      trajectory.computeCoMVelocity(0, 0.0, achievedInitialCoMVelocity);

      FramePoint3D achievedMiddleCoMPosition = new FramePoint3D();
      FrameVector3D achievedMiddleCoMAcceleration = new FrameVector3D();
      trajectory.computeCoMPosition(0, firstDuration, achievedMiddleCoMPosition);
      trajectory.computeCoMAcceleration(0, firstDuration, achievedMiddleCoMAcceleration);
      FramePoint3D achievedMiddleVRP = new FramePoint3D();
      achievedMiddleVRP.scaleAdd(-1.0 / (omega * omega), achievedMiddleCoMAcceleration, achievedMiddleCoMPosition);

      FrameVector3D firstInitialCoMVelocity = new FrameVector3D();
      FrameVector3D firstFinalCoMVelocity = new FrameVector3D();
      FrameVector3D secondInitialVRPVelocity = new FrameVector3D();
      FrameVector3D secondFinalVRPVelocity = new FrameVector3D();

      trajectory.computeVRPVelocity(0, 0.0, firstInitialCoMVelocity);
      trajectory.computeVRPVelocity(0, firstDuration, firstFinalCoMVelocity);
      trajectory.computeVRPVelocity(1, 0.0, secondInitialVRPVelocity);
      trajectory.computeVRPVelocity(1, secondDuration, secondFinalVRPVelocity);

      // check initial continuity
      EuclidCoreTestTools.assertEquals(initialCoM, achievedInitialCoMPosition, 1e-5);
      EuclidCoreTestTools.assertEquals(initialCoMVelocity, achievedInitialCoMVelocity, 1e-1);

      // check smoothness
      EuclidCoreTestTools.assertEquals(firstInitialCoMVelocity, firstFinalCoMVelocity, 1e-4);
      EuclidCoreTestTools.assertEquals(secondInitialVRPVelocity, secondFinalVRPVelocity, 1e-4);

      // check that the smooth velocity is linear
      FrameVector3D secondExpectedVelocity  = new FrameVector3D();
      secondExpectedVelocity.sub(finalDesiredDCM, midpoint);
      secondExpectedVelocity.scale(1.0 / secondDuration);
      EuclidCoreTestTools.assertEquals(secondExpectedVelocity, secondInitialVRPVelocity, 1e-4);

      EuclidCoreTestTools.assertEquals(midpoint, achievedMiddleVRP, 1e-4);
   }
}
