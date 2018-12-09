package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
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

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint2D comPosition = new FramePoint2D();
      planner.setCurrentCoMPosition(comPosition);
      planner.solveForTrajectory();
      planner.compute(0.0);

      FramePoint2D finalICP = new FramePoint2D(secondContact.getCopPosition());

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      planner.compute(1.0);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(finalICP, planner.getDesiredICPPosition(), epsilon);

      FramePoint3D initialICP = new FramePoint3D();
      double interpolation = Math.exp(-omega.getDoubleValue());
      initialICP.interpolate(firstContact.getCopPosition(), secondContact.getCopPosition(), interpolation);

      FramePoint2D initialICP2D = new FramePoint2D(initialICP);

      planner.compute(0.0);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(initialICP2D, planner.getDesiredICPPosition(), epsilon);

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

         EuclidCoreTestTools.assertPoint2DGeometricallyEquals(expectedICP2D, planner.getDesiredICPPosition(), epsilon);
      }
   }

   @Test
   public void testTwoMovingSegmentsInContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, registry);

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

      FramePoint2D comPosition = new FramePoint2D();
      planner.setCurrentCoMPosition(comPosition);
      planner.solveForTrajectory();
      planner.compute(0.0);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3D secondICP = new FramePoint3D();
      FramePoint3D initialICP = new FramePoint3D();
      double interpolation = Math.exp(-omega.getDoubleValue());
      secondICP.interpolate(secondContact.getCopPosition(), thirdContact.getCopPosition(), interpolation);
      initialICP.interpolate(firstContact.getCopPosition(), secondICP, interpolation);

      FramePoint2D initialICP2d = new FramePoint2D(initialICP);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(initialICP2d, planner.getDesiredICPPosition(), epsilon);

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

         EuclidCoreTestTools.assertPoint2DGeometricallyEquals(expectedICP2D, planner.getDesiredICPPosition(), epsilon);
      }
   }

   @Test
   public void testTwoMovingSegmentsOneFlight()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, registry);

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

      FramePoint2D comPosition = new FramePoint2D();
      planner.setCurrentCoMPosition(comPosition);
      planner.solveForTrajectory();
      planner.compute(0.0);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      /*
      FramePoint3D secondICP = new FramePoint3D();
      FramePoint3D initialICP = new FramePoint3D();
      double interpolation = Math.exp(-omega.getDoubleValue());
      secondICP.interpolate(secondContact.getCopPosition(), thirdContact.getCopPosition(), interpolation);
      initialICP.interpolate(firstContact.getCopPosition(), secondICP, interpolation);

      FramePoint2D initialICP2d = new FramePoint2D(initialICP);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(initialICP2d, planner.getDesiredICPPosition(), epsilon);

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

         EuclidCoreTestTools.assertPoint2DGeometricallyEquals(expectedICP2D, planner.getDesiredICPPosition(), epsilon);
      }
      */
   }
}
