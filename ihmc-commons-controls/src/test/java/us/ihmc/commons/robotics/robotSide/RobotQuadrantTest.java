package us.ihmc.commons.robotics.robotSide;


import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class RobotQuadrantTest
{
   private RobotQuadrant frontLeft = RobotQuadrant.FRONT_LEFT;
   private RobotQuadrant frontRight = RobotQuadrant.FRONT_RIGHT;
   private RobotQuadrant hindRight = RobotQuadrant.HIND_RIGHT;
   private RobotQuadrant hindLeft = RobotQuadrant.HIND_LEFT;

   @Test
   public void testGetAcrossBodyLeg()
   {
      assertEquals(frontLeft, frontRight.getAcrossBodyQuadrant());
      assertEquals(frontRight, frontLeft.getAcrossBodyQuadrant());
      assertEquals(hindLeft, hindRight.getAcrossBodyQuadrant());
      assertEquals(hindRight, hindLeft.getAcrossBodyQuadrant());
   }

   @Test
   public void testGetAllLegs()
   {
      ArrayList<RobotQuadrant> actualReturn = RobotQuadrant.getAllQuadrants();
      assertEquals(4, actualReturn.size(), "Number of legs");
      assertEquals(frontLeft, actualReturn.get(0));
      assertEquals(frontRight, actualReturn.get(1));
      assertEquals(hindRight, actualReturn.get(2));
      assertEquals(hindLeft, actualReturn.get(3));
   }

   @Test
   public void testGetDiagonalOppositeLeg()
   {
      assertEquals(hindRight, frontLeft.getDiagonalOppositeQuadrant());
      assertEquals(hindLeft, frontRight.getDiagonalOppositeQuadrant());
      assertEquals(frontLeft, hindRight.getDiagonalOppositeQuadrant());
      assertEquals(frontRight, hindLeft.getDiagonalOppositeQuadrant());
   }

   @Test
   public void testGetLegName()
   {
      assertEquals(frontLeft, RobotQuadrant.getQuadrantName("FRONT_LEFT"));
      assertEquals(frontRight, RobotQuadrant.getQuadrantName("FRONT_RIGHT"));
      assertEquals(hindLeft, RobotQuadrant.getQuadrantName("HIND_LEFT"));
      assertEquals(hindRight, RobotQuadrant.getQuadrantName("HIND_RIGHT"));
   }

   @Test
   public void testGetLegName1()
   {
      assertEquals(frontLeft, RobotQuadrant.getQuadrantNameFromOrdinal(0));
      assertEquals(frontRight, RobotQuadrant.getQuadrantNameFromOrdinal(1));
      assertEquals(hindRight, RobotQuadrant.getQuadrantNameFromOrdinal(2));
      assertEquals(hindLeft, RobotQuadrant.getQuadrantNameFromOrdinal(3));
   }

   @Test
   public void testGetSameSideLeg()
   {
      assertEquals(hindLeft, frontLeft.getSameSideQuadrant());
      assertEquals(hindRight, frontRight.getSameSideQuadrant());
      assertEquals(frontRight, hindRight.getSameSideQuadrant());
      assertEquals(frontLeft, hindLeft.getSameSideQuadrant());
   }

   @Test
   public void testGetShortName()
   {
      assertEquals("FL", frontLeft.getShortName());
      assertEquals("FR", frontRight.getShortName());
      assertEquals("HR", hindRight.getShortName());
      assertEquals("HL", hindLeft.getShortName());
   }

   @Test
   public void testIsLegAFrontLeg()
   {
      assertTrue(frontLeft.isQuadrantInFront());
      assertTrue(frontRight.isQuadrantInFront());
      assertFalse(hindRight.isQuadrantInFront());
      assertFalse(hindLeft.isQuadrantInFront());
   }

   @Test
   public void testIsLegAHindLeg()
   {
      assertFalse(frontLeft.isQuadrantInHind());
      assertFalse(frontRight.isQuadrantInHind());
      assertTrue(hindRight.isQuadrantInHind());
      assertTrue(hindLeft.isQuadrantInHind());
   }

   @Test
   public void testIsLegALeftSideLeg()
   {
      assertTrue(frontLeft.isQuadrantOnLeftSide());
      assertFalse(frontRight.isQuadrantOnLeftSide());
      assertFalse(hindRight.isQuadrantOnLeftSide());
      assertTrue(hindLeft.isQuadrantOnLeftSide());
   }

   @Test
   public void testIsLegARightSideLeg()
   {
      assertFalse(frontLeft.isQuadrantOnRightSide());
      assertTrue(frontRight.isQuadrantOnRightSide());
      assertTrue(hindRight.isQuadrantOnRightSide());
      assertFalse(hindLeft.isQuadrantOnRightSide());
   }
}
