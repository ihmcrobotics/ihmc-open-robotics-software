package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class CoMTrajectoryPlannerTest
{
   private static final double epsilon = 1e-6;
   @Test
   public void testOneMovingSegmentInContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, gravityZ, nominalHeight, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D();
      comPosition.setZ(nominalHeight);
      planner.setCurrentCoMPosition(comPosition);

      planner.solveForTrajectory();
      planner.compute(0.0);

      FramePoint3D finalICP = new FramePoint3D(secondContact.getCopPosition());
      finalICP.addZ(nominalHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      planner.compute(1.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalICP, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialDCM = new FramePoint3D();
      double interpolation = Math.exp(-omega.getDoubleValue());
      initialDCM.interpolate(firstContact.getCopPosition(), secondContact.getCopPosition(), interpolation);
      initialDCM.addZ(nominalHeight);

      planner.compute(0.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D();
      initialVRP.set(firstContact.getCopPosition());
      initialVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         double exponential = Math.exp(omega.getDoubleValue() * time);
         expectedDCM.interpolate(initialVRP, initialDCM, exponential);

         planner.compute(time);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("time : " + time, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testTwoMovingSegmentsInContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, gravityZ, nominalHeight, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();
      QuadrupedContactPhase thirdContact= new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 0.0));
      thirdContact.setTimeInterval(new TimeInterval(2.0, 3.0));
      thirdContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      FramePoint3D comPosition = new FramePoint3D();
      comPosition.setZ(nominalHeight);
      planner.setCurrentCoMPosition(comPosition);
      planner.solveForTrajectory();
      planner.compute(0.0);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3D secondDCM = new FramePoint3D();
      FramePoint3D initialDCM = new FramePoint3D();
      double interpolation = Math.exp(-omega.getDoubleValue());
      secondDCM.interpolate(secondContact.getCopPosition(), thirdContact.getCopPosition(), interpolation);
      initialDCM.interpolate(firstContact.getCopPosition(), secondDCM, interpolation);
      secondDCM.addZ(nominalHeight);
      initialDCM.addZ(nominalHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D(firstContact.getCopPosition());
      initialVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         double exponential = Math.exp(omega.getDoubleValue() * time);
         expectedDCM.interpolate(initialVRP, initialDCM, exponential);

         planner.compute(time);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testTwoMovingSegmentsOneFlight()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, gravityZ, nominalHeight, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();
      QuadrupedContactPhase thirdContact= new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 0.25));
      secondContact.setContactState(ContactState.NO_CONTACT);
      thirdContact.setTimeInterval(new TimeInterval(1.25, 2.25));
      thirdContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      FramePoint3D comPosition = new FramePoint3D();
      comPosition.addZ(nominalHeight);
      planner.setCurrentCoMPosition(comPosition);
      planner.solveForTrajectory();
      planner.compute(0.0);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      /*
      FramePoint3D secondICP = new FramePoint3D();
      FramePoint3D initialICP = new FramePoint3D();
      double interpolation = Math.exp(-omega.getDoubleValue());
      secondICP.interpolate(secondContact.getCopPosition(), thirdContact.getCopPosition(), interpolation);
      initialICP.interpolate(firstContact.getCopPosition(), secondICP, interpolation);

      FramePoint2D initialICP2d = new FramePoint2D(initialICP);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(initialICP2d, planner.getDesiredDCMPosition(), epsilon);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedICP = new FramePoint3D();
         double exponential = Math.exp(omega.getDoubleValue() * time);
         expectedICP.interpolate(firstContact.getCopPosition(), initialICP, exponential);

         FramePoint2D expectedICP2D = new FramePoint2D();
         expectedICP2D.set(expectedICP);

         planner.compute(time);

         EuclidCoreTestTools.assertPoint2DGeometricallyEquals(expectedICP2D, planner.getDesiredDCMPosition(), epsilon);
      }
      */
   }
}
