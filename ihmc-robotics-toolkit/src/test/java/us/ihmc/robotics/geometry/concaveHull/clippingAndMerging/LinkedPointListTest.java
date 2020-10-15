package us.ihmc.robotics.geometry.concaveHull.clippingAndMerging;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging.ConcavePolygon2DClippingTools.LinkedPoint;
import us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging.ConcavePolygon2DClippingTools.LinkedPointList;

import static us.ihmc.robotics.Assert.assertTrue;

public class LinkedPointListTest
{
   @Test
   public void testFourVertexCreationLinkingAndReverse()
   {
      LinkedPointList list = new LinkedPointList();
      LinkedPoint firstLink = new LinkedPoint(-1.0, 1.0);
      LinkedPoint secondLink = new LinkedPoint(1.0, 1.0);
      LinkedPoint thirdLink = new LinkedPoint(1.0, -1.0);
      LinkedPoint fourthLink = new LinkedPoint(-1.0, -1.0);
      list.addPointToEnd(firstLink);
      list.addPointToEnd(secondLink);
      list.addPointToEnd(thirdLink);
      list.addPointToEnd(fourthLink);

      LinkedPoint firstPoint = list.getFirstPoint();
      LinkedPoint lastPoint = list.getLastPoint();

      assertTrue(firstLink.equals(firstPoint));
      assertTrue(fourthLink.equals(lastPoint));

      // check forward loop
      assertTrue(firstLink.getSuccessor().equals(secondLink));
      assertTrue(secondLink.getSuccessor().equals(thirdLink));
      assertTrue(thirdLink.getSuccessor().equals(fourthLink));
      assertTrue(fourthLink.getSuccessor().equals(firstLink));

      // check backwards loop
      assertTrue(fourthLink.getPredecessor().equals(thirdLink));
      assertTrue(thirdLink.getPredecessor().equals(secondLink));
      assertTrue(secondLink.getPredecessor().equals(firstLink));
      assertTrue(firstLink.getPredecessor().equals(fourthLink));

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0, 1.0), firstPoint.getPoint(), 1e-12);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0, -1.0), lastPoint.getPoint(), 1e-12);

      list.reverseOrder();

      assertTrue(list.getFirstPoint().equals(fourthLink));
      assertTrue(list.getLastPoint().equals(firstLink));

      // check forward loop
      assertTrue(fourthLink.getSuccessor().equals(thirdLink));
      assertTrue(thirdLink.getSuccessor().equals(secondLink));
      assertTrue(secondLink.getSuccessor().equals(firstLink));
      assertTrue(firstLink.getSuccessor().equals(fourthLink));

      // check reverse loop
      assertTrue(firstLink.getPredecessor().equals(secondLink));
      assertTrue(secondLink.getPredecessor().equals(thirdLink));
      assertTrue(thirdLink.getPredecessor().equals(fourthLink));
      assertTrue(fourthLink.getPredecessor().equals(firstLink));
   }

   @Test
   public void testThreeVertexCreationLinkingAndReverse()
   {
      LinkedPointList list = new LinkedPointList();
      LinkedPoint firstLink = new LinkedPoint(-1.0, 1.0);
      LinkedPoint secondLink = new LinkedPoint(1.0, 1.0);
      LinkedPoint thirdLink = new LinkedPoint(-1.0, -1.0);
      list.addPointToEnd(firstLink);
      list.addPointToEnd(secondLink);
      list.addPointToEnd(thirdLink);

      LinkedPoint firstPoint = list.getFirstPoint();
      LinkedPoint lastPoint = list.getLastPoint();

      assertTrue(firstLink.equals(firstPoint));
      assertTrue(thirdLink.equals(lastPoint));

      // check forward loop
      assertTrue(firstLink.getSuccessor().equals(secondLink));
      assertTrue(secondLink.getSuccessor().equals(thirdLink));
      assertTrue(thirdLink.getSuccessor().equals(firstLink));

      // check backwards loop
      assertTrue(thirdLink.getPredecessor().equals(secondLink));
      assertTrue(secondLink.getPredecessor().equals(firstLink));
      assertTrue(firstLink.getPredecessor().equals(thirdLink));


      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0, 1.0), firstPoint.getPoint(), 1e-12);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(-1.0, -1.0), lastPoint.getPoint(), 1e-12);
   }

}
