package us.ihmc.robotics.robotSide;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class RobotQuadrantTest
{
   private RobotQuadrant frontLeft = RobotQuadrant.FRONT_LEFT;
   private RobotQuadrant frontRight = RobotQuadrant.FRONT_RIGHT;
   private RobotQuadrant hindRight = RobotQuadrant.HIND_RIGHT;
   private RobotQuadrant hindLeft = RobotQuadrant.HIND_LEFT;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAcrossBodyLeg()
   {
      assertEquals(frontLeft, frontRight.getAcrossBodyQuadrant());
      assertEquals(frontRight, frontLeft.getAcrossBodyQuadrant());
      assertEquals(hindLeft, hindRight.getAcrossBodyQuadrant());
      assertEquals(hindRight, hindLeft.getAcrossBodyQuadrant());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAllLegs()
   {
      ArrayList<RobotQuadrant> actualReturn = RobotQuadrant.getAllQuadrants();
      assertEquals("Number of legs", 4, actualReturn.size());
      assertEquals(frontLeft, actualReturn.get(0));
      assertEquals(frontRight, actualReturn.get(1));
      assertEquals(hindRight, actualReturn.get(2));
      assertEquals(hindLeft, actualReturn.get(3));
   }

//   @DeployableTestMethod(duration = 0.1)
//   @Test(timeout = 30000)
//   public void testGetBodyQuadrant()
//   {
//      ReferenceFrame frame = LittleDogFrames.getBodyFrame();
//      assertEquals(frontLeft, getBodyQuadrant(new FramePoint(frame, 1.0, 1.0, 0)));
//      assertEquals(frontRight, getBodyQuadrant(new FramePoint(frame, 1.0, -1.0, 0)));
//      assertEquals(hindRight, getBodyQuadrant(new FramePoint(frame, -1.0, -1.0, 0)));
//      assertEquals(hindLeft, getBodyQuadrant(new FramePoint(frame, -1.0, 1.0, 0)));
//   }
   
//   private static RobotQuadrant getBodyQuadrant(FramePoint point)
//   {
//      point = new FramePoint(point);
////      point.changeFrame(LittleDogFrames.getBodyFrame());
//
//      if (point.getX() >= 0.0)
//      {
//         if (point.getY() >= 0.0)
//            return RobotQuadrant.FRONT_LEFT;
//         else
//            return RobotQuadrant.FRONT_RIGHT;
//      }
//      else
//      {
//         if (point.getY() >= 0.0)
//            return RobotQuadrant.HIND_LEFT;
//         else
//            return RobotQuadrant.HIND_RIGHT;
//      }
//   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetDiagonalOppositeLeg()
   {
      assertEquals(hindRight, frontLeft.getDiagonalOppositeQuadrant());
      assertEquals(hindLeft, frontRight.getDiagonalOppositeQuadrant());
      assertEquals(frontLeft, hindRight.getDiagonalOppositeQuadrant());
      assertEquals(frontRight, hindLeft.getDiagonalOppositeQuadrant());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetLegName()
   {
      assertEquals(frontLeft, RobotQuadrant.getQuadrantName("FRONT_LEFT"));
      assertEquals(frontRight, RobotQuadrant.getQuadrantName("FRONT_RIGHT"));
      assertEquals(hindLeft, RobotQuadrant.getQuadrantName("HIND_LEFT"));
      assertEquals(hindRight, RobotQuadrant.getQuadrantName("HIND_RIGHT"));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetLegName1()
   {
      assertEquals(frontLeft, RobotQuadrant.getQuadrantNameFromOrdinal(0));
      assertEquals(frontRight, RobotQuadrant.getQuadrantNameFromOrdinal(1));
      assertEquals(hindRight, RobotQuadrant.getQuadrantNameFromOrdinal(2));
      assertEquals(hindLeft, RobotQuadrant.getQuadrantNameFromOrdinal(3));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetSameSideLeg()
   {
      assertEquals(hindLeft, frontLeft.getSameSideQuadrant());
      assertEquals(hindRight, frontRight.getSameSideQuadrant());
      assertEquals(frontRight, hindRight.getSameSideQuadrant());
      assertEquals(frontLeft, hindLeft.getSameSideQuadrant());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetShortName()
   {
      assertEquals("FL", frontLeft.getShortName());
      assertEquals("FR", frontRight.getShortName());
      assertEquals("HR", hindRight.getShortName());
      assertEquals("HL", hindLeft.getShortName());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsLegAFrontLeg()
   {
      assertTrue(frontLeft.isQuadrantInFront());
      assertTrue(frontRight.isQuadrantInFront());
      assertFalse(hindRight.isQuadrantInFront());
      assertFalse(hindLeft.isQuadrantInFront());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsLegAHindLeg()
   {
      assertFalse(frontLeft.isQuadrantInHind());
      assertFalse(frontRight.isQuadrantInHind());
      assertTrue(hindRight.isQuadrantInHind());
      assertTrue(hindLeft.isQuadrantInHind());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsLegALeftSideLeg()
   {
      assertTrue(frontLeft.isQuadrantOnLeftSide());
      assertFalse(frontRight.isQuadrantOnLeftSide());
      assertFalse(hindRight.isQuadrantOnLeftSide());
      assertTrue(hindLeft.isQuadrantOnLeftSide());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsLegARightSideLeg()
   {
      assertFalse(frontLeft.isQuadrantOnRightSide());
      assertTrue(frontRight.isQuadrantOnRightSide());
      assertTrue(hindRight.isQuadrantOnRightSide());
      assertFalse(hindLeft.isQuadrantOnRightSide());
   }
}
