package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.Test;
import org.ojalgo.function.multiary.MultiaryFunction.Convex;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class CopTrajectoryTest
{
   @Test
   public void testTwoSteps()
   {
      Footstep step1 = new Footstep();
      Footstep step2 = new Footstep();

      step1.setRobotSide(RobotSide.LEFT);
      step1.getFootstepPose().getPosition().set(0.25, 0.2, 0.0);
      step2.setRobotSide(RobotSide.RIGHT);
      step2.getFootstepPose().getPosition().set(0.5, -0.2, 0.0);

      double initialTransferTime = 1.0;
      double transferTime = 0.25;
      double swingTime = 1.0;
      FootstepTiming footstepTiming1 = new FootstepTiming(swingTime, initialTransferTime);
      FootstepTiming footstepTiming2 = new FootstepTiming(swingTime, transferTime);

      List<Footstep> steps = new ArrayList<>();
      steps.add(step1);
      steps.add(step2);

      List<FootstepTiming> footstepTimings = new ArrayList<>();
      footstepTimings.add(footstepTiming1);
      footstepTimings.add(footstepTiming2);

      ConvexPolygon2D defaultSupportPolygon = createDefaultSupportPolygon();
      SideDependentList<PoseReferenceFrame> soleFrames = createSoleFrames(0.2);
      SupportSequence supportSequence = new SupportSequence(defaultSupportPolygon, soleFrames, soleFrames);
      CopTrajectory copTrajectory = new CopTrajectory();

      supportSequence.initializeStance();
      supportSequence.update(steps, footstepTimings);
      copTrajectory.update(supportSequence, 1.0);

      ReferenceFrame firstStepFrame = new ReferenceFrame("firstStepFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            step1.getFootstepPose().get(transformToParent);
         }
      };
      firstStepFrame.update();
      ReferenceFrame secondStepFrame = new ReferenceFrame("secondStepFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            step2.getFootstepPose().get(transformToParent);
         }
      };
      secondStepFrame.update();

      FrameConvexPolygon2D firstLeftSupportPolygon = new FrameConvexPolygon2D(soleFrames.get(RobotSide.LEFT), defaultSupportPolygon);
      FrameConvexPolygon2D firstRightSupportPolygon = new FrameConvexPolygon2D(soleFrames.get(RobotSide.RIGHT), defaultSupportPolygon);
      FrameConvexPolygon2D secondLeftSupportPolygon = new FrameConvexPolygon2D(firstStepFrame, defaultSupportPolygon);
      FrameConvexPolygon2D secondRightSupportPolygon = new FrameConvexPolygon2D(secondStepFrame, defaultSupportPolygon);

      firstLeftSupportPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      firstRightSupportPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      secondLeftSupportPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      secondRightSupportPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      ConvexPolygon2D firstTransferSupportPolygon = new ConvexPolygon2D();
      firstTransferSupportPolygon.addVertices(firstLeftSupportPolygon);
      firstTransferSupportPolygon.addVertices(firstRightSupportPolygon);
      firstTransferSupportPolygon.update();
      ConvexPolygon2D firstSwingSupportPolygon = new ConvexPolygon2D();
      firstSwingSupportPolygon.addVertices(firstRightSupportPolygon);
      firstSwingSupportPolygon.update();
      ConvexPolygon2D secondTransferSupportPolygon = new ConvexPolygon2D();
      secondTransferSupportPolygon.addVertices(firstRightSupportPolygon);
      secondTransferSupportPolygon.addVertices(secondLeftSupportPolygon);
      secondTransferSupportPolygon.update();
      ConvexPolygon2D secondSwingSupportPolygon = new ConvexPolygon2D();
      secondSwingSupportPolygon.addVertices(secondLeftSupportPolygon);
      secondSwingSupportPolygon.update();
      ConvexPolygon2D thirdTransferSupportPolygon = new ConvexPolygon2D();
      thirdTransferSupportPolygon.addVertices(secondLeftSupportPolygon);
      thirdTransferSupportPolygon.addVertices(secondRightSupportPolygon);
      thirdTransferSupportPolygon.update();

      assertEquals(6, copTrajectory.getContactStates().size());

      assertEquals(0.0, supportSequence.getSupportTimes().get(0), 1e-7);
      assertEquals(initialTransferTime, supportSequence.getSupportTimes().get(1), 1e-7);
      assertEquals(swingTime + initialTransferTime, supportSequence.getSupportTimes().get(2), 1e-7);
      assertEquals(0.5 * transferTime + swingTime + initialTransferTime, supportSequence.getSupportTimes().get(3), 1e-7);
      assertEquals(transferTime + swingTime + initialTransferTime, supportSequence.getSupportTimes().get(4), 1e-7);
      assertEquals(2.0 * swingTime + transferTime + initialTransferTime, supportSequence.getSupportTimes().get(5), 1e-7);

//      assertEquals(5.25, supportSequence.getSupportTimes().get(5), 1e-7);
      assertTrue(firstTransferSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(0).getCopStartPosition()))); // start
      assertTrue(firstTransferSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(0).getCopEndPosition()))); // end of initial transfer
      assertTrue(firstSwingSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(1).getCopStartPosition()))); // end of initial transfer
      assertTrue(firstSwingSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(1).getCopEndPosition()))); // end of first swing
      assertTrue(secondTransferSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(2).getCopStartPosition()))); // end of first swing
      assertTrue(secondTransferSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(2).getCopEndPosition()))); // halfway through transfer
      assertTrue(secondTransferSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(3).getCopStartPosition()))); // halfway through transfer
      assertTrue(secondTransferSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(3).getCopEndPosition()))); // end of transfer
      assertTrue(secondSwingSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(4).getCopStartPosition()))); // end of transfer
      assertTrue(secondSwingSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(4).getCopEndPosition()))); // end of swing
      assertTrue(thirdTransferSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(5).getCopStartPosition()))); // end of transfer
      assertTrue(thirdTransferSupportPolygon.isPointInside(new Point2D(copTrajectory.getContactStates().get(5).getCopEndPosition()))); // end of swing
   }

   private static SideDependentList<PoseReferenceFrame> createSoleFrames(double width)
   {
      SideDependentList<PoseReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         PoseReferenceFrame soleFrame = new PoseReferenceFrame(robotSide.getLowerCaseName() + "SoleFrame", ReferenceFrame.getWorldFrame());
         soleFrame.setPositionWithoutChecksAndUpdate(0.0, robotSide.negateIfRightSide(width), 0.0);
         soleFrames.put(robotSide, soleFrame);
      }
      return soleFrames;
   }

   private static ConvexPolygon2D createDefaultSupportPolygon()
   {
      ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
      defaultSupportPolygon.addVertex(0.1, 0.05);
      defaultSupportPolygon.addVertex(0.1, -0.05);
      defaultSupportPolygon.addVertex(-0.1, 0.05);
      defaultSupportPolygon.addVertex(-0.1, -0.05);
      defaultSupportPolygon.update();
      return defaultSupportPolygon;
   }
}

