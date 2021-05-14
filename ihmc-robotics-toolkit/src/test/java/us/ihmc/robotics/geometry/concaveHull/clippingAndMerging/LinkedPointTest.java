package us.ihmc.robotics.geometry.concaveHull.clippingAndMerging;

import org.junit.jupiter.api.Test;
import us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging.ConcavePolygon2DClippingTools.LinkedPoint;

import static us.ihmc.robotics.Assert.assertTrue;

public class LinkedPointTest
{
   @Test
   public void testEquals()
   {
      LinkedPoint point1 = new LinkedPoint(0.5, 0.5);
      LinkedPoint point2 = new LinkedPoint(0.5, 0.5);

      assertTrue(point1.equals(point1));
      assertTrue(point2.equals(point1));
      assertTrue(point1.equals(point2));
   }

   @Test
   public void testSettingAndGettingLinks()
   {
      LinkedPoint pointToTest = new LinkedPoint(0.5, 0.5);
      LinkedPoint successor = new LinkedPoint(1.0, 0.0);
      LinkedPoint predecessor= new LinkedPoint(0.0, 0.5);
      pointToTest.setSuccessor(successor);
      pointToTest.setPredecessor(predecessor);

      assertTrue(pointToTest.getSuccessor().equals(successor));
      assertTrue(pointToTest.getPredecessor().equals(predecessor));
   }

}
