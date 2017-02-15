package us.ihmc.humanoidRobotics.footstep.footstepGenerator.overheadPath;

import static org.junit.Assert.assertEquals;

import javax.vecmath.Point2d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.MemoryTools;

public class OverheadPathTest
{
   private boolean VERBOSE = false;
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void turningOverheadPath_UsageTest()
   {
      double eps = 1e-15;
      FramePoint2d startPosition = new FramePoint2d(WORLD_FRAME, 1.0, 2.0);
      double startYaw = Math.PI / 4;
      FrameOrientation2d startOrientation = new FrameOrientation2d(WORLD_FRAME, startYaw);
      double endYaw = Math.PI * 3 / 4;
      FrameOrientation2d endOrientation = new FrameOrientation2d(WORLD_FRAME, endYaw);
      FramePose2d startPose = new FramePose2d(startPosition, startOrientation);
      TurningOverheadPath turningPath = new TurningOverheadPath(startPose, endOrientation);

      assertEquals("StartYaw", startYaw, turningPath.getPoseAtS(0).getYaw(), eps);
      assertEquals("EndYaw", endYaw, turningPath.getPoseAtS(1).getYaw(), eps);
      assertEquals("DeltaYaw", endYaw - startYaw, turningPath.getDeltaYaw(), eps);

      FramePose2d framePose2d;
      for (double s = 0; s <= 1.0; s += 0.1)
      {
         framePose2d = turningPath.getPoseAtS(s);
         if (VERBOSE)
            System.out.println(s + " " + framePose2d.getYaw());
         assertEquals("XPosition wrong at s = " + s, startPosition.getX(), framePose2d.getX(), eps);
         assertEquals("YPosition wrong at s = " + s, startPosition.getY(), framePose2d.getY(), eps);
         double expectedYaw = (1 - s) * startYaw + s * endYaw;
         assertEquals("Yaw wrong at s = " + s, expectedYaw, framePose2d.getYaw(), eps);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void turningOverheadPath_PassingPiTest()
   {
      double eps = 1e-15;
      FramePoint2d startPosition = new FramePoint2d(WORLD_FRAME, 1.0, 2.0);
      double startYaw = 3 * Math.PI / 4;
      FrameOrientation2d startOrientation = new FrameOrientation2d(WORLD_FRAME, startYaw);
      double endYaw = -startYaw;
      FrameOrientation2d endOrientation = new FrameOrientation2d(WORLD_FRAME, endYaw);
      FramePose2d startPose = new FramePose2d(startPosition, startOrientation);
      TurningOverheadPath turningPath = new TurningOverheadPath(startPose, endOrientation);

      assertEquals("StartYaw", startYaw, turningPath.getPoseAtS(0).getYaw(), eps);
      assertEquals("EndYaw", endYaw, turningPath.getPoseAtS(1).getYaw(), eps);
      assertEquals("DeltaYaw", 2 * (Math.PI - startYaw), turningPath.getDeltaYaw(), eps);

      FramePose2d framePose2d;
      for (double s = 0; s <= 1.0; s += 0.1)
      {
         framePose2d = turningPath.getPoseAtS(s);
         if (VERBOSE)
            System.out.println(s + " " + framePose2d.getYaw());
         assertEquals("XPosition wrong at s = " + s, startPosition.getX(), framePose2d.getX(), eps);
         assertEquals("YPosition wrong at s = " + s, startPosition.getY(), framePose2d.getY(), eps);

         // Yaw goes in positive direction across pi and jumps from pi to -pi, continuing to move in positive direction.
         double expectedYaw = (1 - s) * (startYaw) + s * (startYaw + 2 * (Math.PI - startYaw)) - 2 * Math.PI * ((s >= 0.5) ? 1 : 0);
         assertEquals("Yaw wrong at s = " + s, expectedYaw, framePose2d.getYaw(), eps);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void straightLineOverheadPath_UsageTest()
   {
      double eps = 1e-15;
      final double x1 = 1.0;
      final double y1 = 2.0;
      final double x2 = 5.0;
      final double y2 = 9.0;
      final double yaw = Math.PI / 4;

      FramePoint2d startPosition = new FramePoint2d(WORLD_FRAME, x1, y1);
      FramePoint2d endPosition = new FramePoint2d(WORLD_FRAME, x2, y2);
      FrameOrientation2d orientation = new FrameOrientation2d(WORLD_FRAME, yaw);
      FramePose2d startPose = new FramePose2d(startPosition, orientation);
      StraightLineOverheadPath straightPath = new StraightLineOverheadPath(startPose, endPosition);

      assertEquals("StartYaw", yaw, straightPath.getPoseAtS(0).getYaw(), eps);
      assertEquals("EndYaw", yaw, straightPath.getPoseAtS(1).getYaw(), eps);
      assertEquals("StartX", x1, straightPath.getPoseAtS(0).getX(), eps);
      assertEquals("EndX", x2, straightPath.getPoseAtS(1).getX(), eps);
      assertEquals("StartY", y1, straightPath.getPoseAtS(0).getY(), eps);
      assertEquals("EndY", y2, straightPath.getPoseAtS(1).getY(), eps);

      FramePose2d framePose2d;
      for (double s = 0; s <= 1.0; s += 0.1)
      {
         framePose2d = straightPath.getPoseAtS(s);
         if (VERBOSE)
            System.out.println("s" + s + " x" + framePose2d.getX() + " y" + framePose2d.getY() + " yaw" + framePose2d.getYaw());

         double expectedX = (1 - s) * x1 + s * x2;
         assertEquals("XPosition wrong at s = " + s, expectedX, framePose2d.getX(), eps);
         double expectedY = (1 - s) * y1 + s * y2;
         assertEquals("YPosition wrong at s = " + s, expectedY, framePose2d.getY(), eps);
         double expectedYaw = yaw;
         assertEquals("Yaw wrong at s = " + s, expectedYaw, framePose2d.getYaw(), eps);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void turnThenStraightOverheadPath_UsageAndHeadingOffsetTest()
   {
      double eps = 1e-14;
      final double x1 = 1.0;
      final double y1 = 2.0;
      final double x2 = 5.0;
      final double y2 = 9.0;
      final double startYaw = Math.PI / 4;

      FramePoint2d startPosition = new FramePoint2d(WORLD_FRAME, x1, y1);
      FramePoint2d endPosition = new FramePoint2d(WORLD_FRAME, x2, y2);
      FrameOrientation2d orientation = new FrameOrientation2d(WORLD_FRAME, startYaw);
      FramePose2d startPose = new FramePose2d(startPosition, orientation);

      for (double headingOffset = -Math.PI; headingOffset < Math.PI; headingOffset += Math.PI / 2)    // Direction facing relative to the path direction while traveling on the straight line portion of the path
      {
         TurnThenStraightOverheadPath path = new TurnThenStraightOverheadPath(startPose, endPosition, headingOffset);

         double endYawExpected = Math.atan2(y2 - y1, x2 - x1) + headingOffset;
         if (VERBOSE)
            System.out.println("endYawExpected: " + endYawExpected);
         assertEquals("BegYaw", startYaw, path.getPoseAtS(0).getYaw(), eps);
         assertEquals("MidYaw", endYawExpected, path.getPoseAtS(0.5).getYaw(), eps);
         assertEquals("EndYaw", endYawExpected, path.getPoseAtS(1).getYaw(), eps);
         assertEquals("BegX", x1, path.getPoseAtS(0).getX(), eps);
         assertEquals("MidX", x1, path.getPoseAtS(0.5).getX(), eps);
         assertEquals("EndX", x2, path.getPoseAtS(1).getX(), eps);
         assertEquals("BegY", y1, path.getPoseAtS(0).getY(), eps);
         assertEquals("MidY", y1, path.getPoseAtS(0.5).getY(), eps);
         assertEquals("EndY", y2, path.getPoseAtS(1).getY(), eps);

         FramePose2d framePose2d;
         double smid = 0.5;
         for (double s = 0; s <= smid; s += 0.1)
         {
            framePose2d = path.getPoseAtS(s);
            if (VERBOSE)
               System.out.print("s: " + s + " x: " + framePose2d.getX() + " y: " + framePose2d.getY() + " yaw: " + framePose2d.getYaw());

            assertEquals("XPosition wrong at s = " + s, x1, framePose2d.getX(), eps);
            assertEquals("YPosition wrong at s = " + s, y1, framePose2d.getY(), eps);
            double expectedYaw = (smid - s) / smid * startYaw + s / smid * endYawExpected;    // For this test this is true, but this is not a general condition as tested by the turning path crossing pi case above.
            if (VERBOSE)
               System.out.println(" expectedYaw: " + expectedYaw);

            assertEquals("Yaw wrong at s = " + s, expectedYaw, framePose2d.getYaw(), eps);
         }

         for (double s2 = 0; s2 <= 1.0; s2 += 0.1)
         {
            double s = (s2 + 1) * smid;
            framePose2d = path.getPoseAtS(s);
            if (VERBOSE)
               System.out.println("s: " + s + " " + s2 + " x: " + framePose2d.getX() + " y: " + framePose2d.getY() + " yaw: " + framePose2d.getYaw());

            double expectedX = (1 - s2) * x1 + s2 * x2;
            assertEquals("XPosition wrong at s = " + s, expectedX, framePose2d.getX(), eps);
            double expectedY = (1 - s2) * y1 + s2 * y2;
            assertEquals("YPosition wrong at s = " + s, expectedY, framePose2d.getY(), eps);
            assertEquals("Yaw wrong at s = " + s, endYawExpected, framePose2d.getYaw(), eps);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void turnStraightTurnOverheadPath_UsageAndHeadingOffsetTest()
   {
      double eps = 1e-14;
      final double x1 = 1.0;
      final double y1 = 2.0;
      final double x2 = 5.0;
      final double y2 = 9.0;
      final double startYaw = Math.PI / 4;
      final double endYaw = -Math.PI / 8;    // This value chosen just for "simple" test to avoid the crossing pi logic (which should be handled by the turning path/crossing pi test above)

      FramePoint2d startPosition = new FramePoint2d(WORLD_FRAME, x1, y1);
      FramePoint2d endPosition = new FramePoint2d(WORLD_FRAME, x2, y2);
      FrameOrientation2d startOrientation = new FrameOrientation2d(WORLD_FRAME, startYaw);
      FrameOrientation2d endOrientation = new FrameOrientation2d(WORLD_FRAME, endYaw);
      FramePose2d startPose = new FramePose2d(startPosition, startOrientation);
      FramePose2d endPose = new FramePose2d(endPosition, endOrientation);

      for (double headingOffset = -Math.PI; headingOffset < Math.PI; headingOffset += Math.PI / 2)    // Direction facing relative to the path direction while traveling on the straight line portion of the path
      {
         TurnStraightTurnOverheadPath path = new TurnStraightTurnOverheadPath(startPose, endPose, headingOffset);

         double pathYawExpected = Math.atan2(y2 - y1, x2 - x1) + headingOffset;
         double sTrans1 = 1.0 / 3;
         double sTrans2 = 2.0 / 3;
         double sMidTurn1 = 1.0 / 6;
         double sMidTurn2 = 5.0 / 6;
         if (VERBOSE)
         {
            System.out.println("headingOffset: " + headingOffset);
            System.out.println(" startYaw: " + startYaw);
            System.out.println(" pathYawExpected: " + pathYawExpected);
            System.out.println(" endYaw: " + endYaw);
         }

         assertEquals("BegYaw", startYaw, path.getPoseAtS(0).getYaw(), eps);
         assertEquals("MidTurn1Yaw", (startYaw + pathYawExpected) / 2, path.getPoseAtS(sMidTurn1).getYaw(), eps);
         assertEquals("Trans1Yaw", pathYawExpected, path.getPoseAtS(sTrans1).getYaw(), eps);
         assertEquals("MidYaw", pathYawExpected, path.getPoseAtS(0.5).getYaw(), eps);
         assertEquals("Trans2Yaw", pathYawExpected, path.getPoseAtS(sTrans2).getYaw(), eps);
         assertEquals("MidTurn2Yaw", (endYaw + pathYawExpected) / 2, path.getPoseAtS(sMidTurn2).getYaw(), eps);
         assertEquals("EndYaw", endYaw, path.getPoseAtS(1).getYaw(), eps);

         assertEquals("BegX", x1, path.getPoseAtS(0).getX(), eps);
         assertEquals("MidTurn1X", x1, path.getPoseAtS(sMidTurn1).getX(), eps);
         assertEquals("Trans1X", x1, path.getPoseAtS(sTrans1).getX(), eps);
         assertEquals("MidX", (x1 + x2) / 2, path.getPoseAtS(0.5).getX(), eps);
         assertEquals("Trans2X", x2, path.getPoseAtS(sTrans2).getX(), eps);
         assertEquals("MidTurn2X", x2, path.getPoseAtS(sMidTurn2).getX(), eps);
         assertEquals("EndX", x2, path.getPoseAtS(1).getX(), eps);

         assertEquals("BegY", y1, path.getPoseAtS(0).getY(), eps);
         assertEquals("MidTurn1Y", y1, path.getPoseAtS(sMidTurn1).getY(), eps);
         assertEquals("Trans1Y", y1, path.getPoseAtS(sTrans1).getY(), eps);
         assertEquals("MidY", (y1 + y2) / 2, path.getPoseAtS(0.5).getY(), eps);
         assertEquals("Trans2Y", y2, path.getPoseAtS(sTrans2).getY(), eps);
         assertEquals("MidTurn2Y", y2, path.getPoseAtS(sMidTurn2).getY(), eps);
         assertEquals("EndY", y2, path.getPoseAtS(1).getY(), eps);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void turnStraightTurnOverheadPath_turnInPlaceTest()
   {
      // Turn in place would try to go to zero yaw for the intermediate case which could cause it to start to turn away from
      // the desired angle first and then turn back, or even cause large jumps!
      double eps = 1e-14;
      final double x1 = 1.0;
      final double y1 = 2.0;
      final double x2 = x1;
      final double y2 = y1;
      final double startYaw = Math.toRadians(30);    // 30 to 45 in place was the first case found that did something (obviously) wrong for the footstep generators...

      FramePoint2d startPosition = new FramePoint2d(WORLD_FRAME, x1, y1);
      FramePoint2d endPosition = new FramePoint2d(WORLD_FRAME, x2, y2);
      FrameOrientation2d startOrientation = new FrameOrientation2d(WORLD_FRAME, startYaw);
      FramePose2d startPose = new FramePose2d(startPosition, startOrientation);
      for (double endYaw = 0; endYaw < 2 * Math.PI; endYaw += Math.PI / 8)
      {
         FrameOrientation2d endOrientation = new FrameOrientation2d(WORLD_FRAME, endYaw);
         FramePose2d endPose = new FramePose2d(endPosition, endOrientation);

         // headingOffset is the direction facing relative to the path direction while traveling on the straight line portion of the path
         // e.g. 0 is forwards, -Math.PI is backwards, Math.PI/2 is facing left/walking to the right.
         for (double headingOffset = -Math.PI; headingOffset < Math.PI; headingOffset += Math.PI / 2)
         {
            TurnStraightTurnOverheadPath path = new TurnStraightTurnOverheadPath(startPose, endPose, headingOffset);

            // pathYaw here is actually useless, since there is no straight portion of the path!
            // double pathYawExpected = Math.atan2(y2 - y1, x2 - x1) + headingOffset;
            double pathYawExpected = startYaw;    // When turning in place, it should be the same as what started
            double sTrans1 = 1.0 / 3;
            double sTrans2 = 2.0 / 3;
            double sMidTurn1 = 1.0 / 6;
            double sMidTurn2 = 5.0 / 6;
            VERBOSE = true;

            if (VERBOSE)
            {
               System.out.println("headingOffset: " + headingOffset);
               System.out.println(" startYaw: " + startYaw);
               System.out.println(" pathYawExpected: " + pathYawExpected);
               System.out.println(" endYaw: " + endYaw);
            }

            String preMessage = startYaw + " to " + endYaw + ". ";
            assertEquals(preMessage + "BegYaw", startYaw, path.getPoseAtS(0).getYaw(), eps);
            assertEquals(preMessage + "MidTurn1Yaw", (startYaw + pathYawExpected) / 2, path.getPoseAtS(sMidTurn1).getYaw(), eps);
            assertEquals(preMessage + "Trans1Yaw", pathYawExpected, path.getPoseAtS(sTrans1).getYaw(), eps);
            assertEquals(preMessage + "MidYaw", pathYawExpected, path.getPoseAtS(0.5).getYaw(), eps);
            assertEquals(preMessage + "Trans2Yaw", pathYawExpected, path.getPoseAtS(sTrans2).getYaw(), eps);
            double turnDirection = AngleTools.trimAngleMinusPiToPi(endYaw - startYaw);
            assertEquals(preMessage + "MidTurn2Yaw", AngleTools.trimAngleMinusPiToPi((startYaw + (startYaw + turnDirection)) / 2),
                         path.getPoseAtS(sMidTurn2).getYaw(), eps);
            assertEquals(preMessage + "EndYaw", AngleTools.trimAngleMinusPiToPi(endYaw), path.getPoseAtS(1).getYaw(), eps);

            assertEquals(preMessage + "BegX", x1, path.getPoseAtS(0).getX(), eps);
            assertEquals(preMessage + "MidTurn1X", x1, path.getPoseAtS(sMidTurn1).getX(), eps);
            assertEquals(preMessage + "Trans1X", x1, path.getPoseAtS(sTrans1).getX(), eps);
            assertEquals(preMessage + "MidX", (x1 + x2) / 2, path.getPoseAtS(0.5).getX(), eps);
            assertEquals(preMessage + "Trans2X", x2, path.getPoseAtS(sTrans2).getX(), eps);
            assertEquals(preMessage + "MidTurn2X", x2, path.getPoseAtS(sMidTurn2).getX(), eps);
            assertEquals(preMessage + "EndX", x2, path.getPoseAtS(1).getX(), eps);

            assertEquals(preMessage + "BegY", y1, path.getPoseAtS(0).getY(), eps);
            assertEquals(preMessage + "MidTurn1Y", y1, path.getPoseAtS(sMidTurn1).getY(), eps);
            assertEquals(preMessage + "Trans1Y", y1, path.getPoseAtS(sTrans1).getY(), eps);
            assertEquals(preMessage + "MidY", (y1 + y2) / 2, path.getPoseAtS(0.5).getY(), eps);
            assertEquals(preMessage + "Trans2Y", y2, path.getPoseAtS(sTrans2).getY(), eps);
            assertEquals(preMessage + "MidTurn2Y", y2, path.getPoseAtS(sMidTurn2).getY(), eps);
            assertEquals(preMessage + "EndY", y2, path.getPoseAtS(1).getY(), eps);
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void turningPathInterpolationExtrapolationTest()
   {
      double eps = 1e-14;
      
      FramePose2d startPose = new FramePose2d(WORLD_FRAME);
      FrameOrientation2d endPosition = new FrameOrientation2d(WORLD_FRAME, 1.0);
      TurningOverheadPath p = new TurningOverheadPath(startPose, endPosition);
      
      FramePose2d poseAtS = p.getPoseAtS(2.0);
      assertEquals("Should cap at 1.0", 1.0, poseAtS.getYaw(), eps);
      
      poseAtS = p.getExtrapolatedPoseAtS(2.0);
      assertEquals("Should extrapolate to 2.0", 2.0, poseAtS.getYaw(), eps);
      
      poseAtS = p.getPoseAtS(-1.0);
      assertEquals("Should cap at 0.0", 0.0, poseAtS.getYaw(), eps);
      
      poseAtS = p.getExtrapolatedPoseAtS(-1.0);
      assertEquals("Should extrapolate to -1.0", -1.0,poseAtS.getYaw(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
	@Test(timeout = 30000)
   public void straightPathInterpolationExtrapolationTest()
   {
      double eps = 1e-14;

      FramePose2d startPose = new FramePose2d(WORLD_FRAME);
      Point2d position = new Point2d(1.0, 0.0);
      FramePoint2d endPosition = new FramePoint2d(WORLD_FRAME, position);
      StraightLineOverheadPath p = new StraightLineOverheadPath(startPose, endPosition);
      
      
      FramePose2d poseAtS = p.getPoseAtS(2.0);
      assertEquals("Should cap at 1.0", 1.0, poseAtS.getX(), eps);
      
      poseAtS = p.getExtrapolatedPoseAtS(2.0);
      assertEquals("Should extrapolate to 2.0", 2.0, poseAtS.getX(), eps);
      
      poseAtS = p.getPoseAtS(-1.0);
      assertEquals("Should cap at 0.0", 0.0, poseAtS.getX(), eps);
      
      poseAtS = p.getExtrapolatedPoseAtS(-1.0);
      assertEquals("Should extrapolate to -1.0", -1.0,poseAtS.getX(), eps);
   }
}
