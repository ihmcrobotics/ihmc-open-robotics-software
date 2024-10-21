package us.ihmc.robotics.robotSide;


import java.util.ArrayList;

import org.junit.jupiter.api.Assertions;
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
      Assertions.assertEquals(frontLeft, frontRight.getAcrossBodyQuadrant());
      Assertions.assertEquals(frontRight, frontLeft.getAcrossBodyQuadrant());
      Assertions.assertEquals(hindLeft, hindRight.getAcrossBodyQuadrant());
      Assertions.assertEquals(hindRight, hindLeft.getAcrossBodyQuadrant());
   }

   @Test
   public void testGetAllLegs()
   {
      ArrayList<RobotQuadrant> actualReturn = RobotQuadrant.getAllQuadrants();
      assertEquals(4, actualReturn.size(), "Number of legs");
      Assertions.assertEquals(frontLeft, actualReturn.get(0));
      Assertions.assertEquals(frontRight, actualReturn.get(1));
      Assertions.assertEquals(hindRight, actualReturn.get(2));
      Assertions.assertEquals(hindLeft, actualReturn.get(3));
   }

   @Test
   public void testGetDiagonalOppositeLeg()
   {
      Assertions.assertEquals(hindRight, frontLeft.getDiagonalOppositeQuadrant());
      Assertions.assertEquals(hindLeft, frontRight.getDiagonalOppositeQuadrant());
      Assertions.assertEquals(frontLeft, hindRight.getDiagonalOppositeQuadrant());
      Assertions.assertEquals(frontRight, hindLeft.getDiagonalOppositeQuadrant());
   }

   @Test
   public void testGetLegName()
   {
      Assertions.assertEquals(frontLeft, RobotQuadrant.getQuadrantName("FRONT_LEFT"));
      Assertions.assertEquals(frontRight, RobotQuadrant.getQuadrantName("FRONT_RIGHT"));
      Assertions.assertEquals(hindLeft, RobotQuadrant.getQuadrantName("HIND_LEFT"));
      Assertions.assertEquals(hindRight, RobotQuadrant.getQuadrantName("HIND_RIGHT"));
   }

   @Test
   public void testGetLegName1()
   {
      Assertions.assertEquals(frontLeft, RobotQuadrant.getQuadrantNameFromOrdinal(0));
      Assertions.assertEquals(frontRight, RobotQuadrant.getQuadrantNameFromOrdinal(1));
      Assertions.assertEquals(hindRight, RobotQuadrant.getQuadrantNameFromOrdinal(2));
      Assertions.assertEquals(hindLeft, RobotQuadrant.getQuadrantNameFromOrdinal(3));
   }

   @Test
   public void testGetSameSideLeg()
   {
      Assertions.assertEquals(hindLeft, frontLeft.getSameSideQuadrant());
      Assertions.assertEquals(hindRight, frontRight.getSameSideQuadrant());
      Assertions.assertEquals(frontRight, hindRight.getSameSideQuadrant());
      Assertions.assertEquals(frontLeft, hindLeft.getSameSideQuadrant());
   }

   @Test
   public void testGetShortName()
   {
      Assertions.assertEquals("FL", frontLeft.getShortName());
      Assertions.assertEquals("FR", frontRight.getShortName());
      Assertions.assertEquals("HR", hindRight.getShortName());
      Assertions.assertEquals("HL", hindLeft.getShortName());
   }

   @Test
   public void testIsLegAFrontLeg()
   {
      Assertions.assertTrue(frontLeft.isQuadrantInFront());
      Assertions.assertTrue(frontRight.isQuadrantInFront());
      Assertions.assertFalse(hindRight.isQuadrantInFront());
      Assertions.assertFalse(hindLeft.isQuadrantInFront());
   }

   @Test
   public void testIsLegAHindLeg()
   {
      Assertions.assertFalse(frontLeft.isQuadrantInHind());
      Assertions.assertFalse(frontRight.isQuadrantInHind());
      Assertions.assertTrue(hindRight.isQuadrantInHind());
      Assertions.assertTrue(hindLeft.isQuadrantInHind());
   }

   @Test
   public void testIsLegALeftSideLeg()
   {
      Assertions.assertTrue(frontLeft.isQuadrantOnLeftSide());
      Assertions.assertFalse(frontRight.isQuadrantOnLeftSide());
      Assertions.assertFalse(hindRight.isQuadrantOnLeftSide());
      Assertions.assertTrue(hindLeft.isQuadrantOnLeftSide());
   }

   @Test
   public void testIsLegARightSideLeg()
   {
      Assertions.assertFalse(frontLeft.isQuadrantOnRightSide());
      Assertions.assertTrue(frontRight.isQuadrantOnRightSide());
      Assertions.assertTrue(hindRight.isQuadrantOnRightSide());
      Assertions.assertFalse(hindLeft.isQuadrantOnRightSide());
   }
}
