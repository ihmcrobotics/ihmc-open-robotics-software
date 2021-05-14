package us.ihmc.robotics.occupancyGrid;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class OccupancyGridToolsTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testRegisteringFourPoints()
   {
      FramePoint2D pointA = new FramePoint2D(worldFrame, 1.0, 1.0);
      FramePoint2D pointB = new FramePoint2D(worldFrame, -1.0, 1.0);
      FramePoint2D pointC = new FramePoint2D(worldFrame, -1.0, -1.0);
      FramePoint2D pointD = new FramePoint2D(worldFrame, 1.0, -1.0);

      ConvexPolygon2D expectedHull = new ConvexPolygon2D();
      expectedHull.addVertex(pointA);
      expectedHull.addVertex(pointB);
      expectedHull.addVertex(pointC);
      expectedHull.addVertex(pointD);
      expectedHull.update();

      OccupancyGrid occupancyGrid = new OccupancyGrid("", worldFrame, new YoRegistry("test"));
      occupancyGrid.registerPoint(pointA);
      occupancyGrid.registerPoint(pointA);
      occupancyGrid.registerPoint(pointB);
      occupancyGrid.registerPoint(pointC);
      occupancyGrid.registerPoint(pointC);
      occupancyGrid.registerPoint(pointD);

      FrameConvexPolygon2D hull = new FrameConvexPolygon2D();
      OccupancyGridTools.computeConvexHullOfOccupancyGrid(occupancyGrid, hull);

      assertEquals(4, occupancyGrid.getNumberOfOccupiedCells());
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(expectedHull, hull, 1e-8);

      double decayRate = 0.1;
      int ticksToBarelyOccupied = (int) Math.floor(Math.log(0.5) / Math.log(1.0 - decayRate));
      int ticksToHalfOccupied = (int) Math.floor(Math.log(1.0) / Math.log(1.0 - decayRate));
      occupancyGrid.setOccupancyDecayRate(decayRate);
      int ticks = 0;
      for (; ticks < ticksToHalfOccupied; ticks++)
         occupancyGrid.update();

      OccupancyGridTools.computeConvexHullOfOccupancyGrid(occupancyGrid, hull);

      assertEquals(4, occupancyGrid.getNumberOfOccupiedCells());
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(expectedHull, hull, 1e-8);

      expectedHull.clear();
      expectedHull.addVertex(pointA);
      expectedHull.addVertex(pointC);
      expectedHull.update();

      occupancyGrid.update();
      ticks++;

      OccupancyGridTools.computeConvexHullOfOccupancyGrid(occupancyGrid, hull);

      assertEquals(2, occupancyGrid.getNumberOfOccupiedCells());
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(expectedHull, hull, 1e-8);

      for (; ticks < ticksToBarelyOccupied; ticks++)
         occupancyGrid.update();

      OccupancyGridTools.computeConvexHullOfOccupancyGrid(occupancyGrid, hull);

      assertEquals(2, occupancyGrid.getNumberOfOccupiedCells());
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(expectedHull, hull, 1e-8);

      occupancyGrid.update();

      expectedHull.clear();
      OccupancyGridTools.computeConvexHullOfOccupancyGrid(occupancyGrid, hull);

      assertEquals(0, occupancyGrid.getNumberOfOccupiedCells());
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(expectedHull, hull, 1e-8);
   }

   @Test
   public void testRandomHull()
   {
      OccupancyGrid occupancyGrid = new OccupancyGrid("", worldFrame, new YoRegistry("test"));
      ConvexPolygon2D polygon2D = new ConvexPolygon2D();

      Random random = new Random(1738L);
      int pointsToAdd = 100;
      for (int i = 0; i < pointsToAdd; i++)
      {
         FramePoint2D point = EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame);
         point.setX(OccupancyGrid.findIndex(point.getX(), OccupancyGrid.defaultCellSize));
         point.setY(OccupancyGrid.findIndex(point.getY(), OccupancyGrid.defaultCellSize));
         polygon2D.addVertex(point);
         occupancyGrid.registerPoint(point);
      }
      polygon2D.update();
      FrameConvexPolygon2D hull = new FrameConvexPolygon2D();
      OccupancyGridTools.computeConvexHullOfOccupancyGrid(occupancyGrid, hull);
      EuclidGeometryTestTools.assertConvexPolygon2DEquals(polygon2D, hull, 1e-8);
   }


   @Test
   public void testCellsOnSideOfLine()
   {
      OccupancyGrid occupancyGrid = new OccupancyGrid("", worldFrame, new YoRegistry("test"));
      FrameLine2D verticalLine = new FrameLine2D();
      FrameLine2D horizontalLine = new FrameLine2D();
      verticalLine.getDirection().set(1.0, 0.0);
      horizontalLine.getDirection().set(0.0, 1.0);

      occupancyGrid.registerPoint(new FramePoint2D(worldFrame, 0.0, 1.0));

      int onLeftSide = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.LEFT, 0.0);
      int onRightSide = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.RIGHT, 0.0);
      int above = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, horizontalLine, RobotSide.RIGHT, 0.0);
      int below = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, horizontalLine, RobotSide.LEFT, 0.0);

      assertEquals(1, onLeftSide);
      assertEquals(0, onRightSide);
      assertEquals(0, above);
      assertEquals(0, below);

      occupancyGrid.registerPoint(new FramePoint2D(worldFrame, 0.0, -1.0));

      onLeftSide = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.LEFT, 0.0);
      onRightSide = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.RIGHT, 0.0);
      above = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, horizontalLine, RobotSide.RIGHT, 0.0);
      below = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, horizontalLine, RobotSide.LEFT, 0.0);

      assertEquals(1, onLeftSide);
      assertEquals(1, onRightSide);
      assertEquals(0, above);
      assertEquals(0, below);

      occupancyGrid.registerPoint(new FramePoint2D(worldFrame, 1.0, 0.0));

      onLeftSide = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.LEFT, 0.0);
      onRightSide = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.RIGHT, 0.0);
      above = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, horizontalLine, RobotSide.RIGHT, 0.0);
      below = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, horizontalLine, RobotSide.LEFT, 0.0);

      assertEquals(1, onLeftSide);
      assertEquals(1, onRightSide);
      assertEquals(1, above);
      assertEquals(0, below);

      occupancyGrid.registerPoint(new FramePoint2D(worldFrame, -1.0, 0.0));

      onLeftSide = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.LEFT, 0.0);
      onRightSide = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.RIGHT, 0.0);
      above = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, horizontalLine, RobotSide.RIGHT, 0.0);
      below = OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, horizontalLine, RobotSide.LEFT, 0.0);

      assertEquals(1, onLeftSide);
      assertEquals(1, onRightSide);
      assertEquals(1, above);
      assertEquals(1, below);


      assertEquals(1, OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.LEFT, 0.95));
      assertEquals(1, OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.LEFT, 0.99));
      assertEquals(1, OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.LEFT, 1.0 - 1e-4));
      assertEquals(0, OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.LEFT, 1.0));
      assertEquals(0, OccupancyGridTools.computeNumberOfCellsOccupiedOnSideOfLine(occupancyGrid, verticalLine, RobotSide.LEFT, 1.5));
   }
}
