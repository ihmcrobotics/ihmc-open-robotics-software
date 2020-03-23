package us.ihmc.robotics.occupancyGrid;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public class OccupancyGridTools
{
   public static void computeConvexHullOfOccupancyGrid(OccupancyGrid occupancyGrid, FrameConvexPolygon2D convexHullToPack)
   {
      convexHullToPack.clear(occupancyGrid.getGridFrame());

      List<OccupancyGridCell> activeCells = occupancyGrid.getAllActiveCells();
      for (int i = 0; i < activeCells.size(); i++)
      {
         OccupancyGridCell cell = activeCells.get(i);
         if (cell.getIsOccupied())
            convexHullToPack.addVertex(occupancyGrid.getXLocation(cell.getXIndex()), occupancyGrid.getYLocation(cell.getYIndex()));
      }

      convexHullToPack.update();
   }

   public static int computeNumberOfCellsOccupiedOnSideOfLine(OccupancyGrid occupancyGrid,
                                                              FrameLine2DReadOnly frameLine,
                                                              RobotSide sideToLookAt,
                                                              double minDistanceFromLine)
   {
      ReferenceFrame gridFrame = occupancyGrid.getGridFrame();

      // First create a shifted line towards the sideToLookAt such that we don't check the cells for which the line goes through.
      frameLine.checkReferenceFrameMatch(gridFrame);

      // The shiftingVector is used to shift the line.
      // We first make it perpendicular to the line, normal, and pointing towards the sideToLookAt.
      double shiftingVectorX = -frameLine.getDirectionY();
      double shiftingVectorY = frameLine.getDirectionX();
      if (sideToLookAt == RobotSide.RIGHT)
      {
         shiftingVectorX = -shiftingVectorX;
         shiftingVectorY = -shiftingVectorY;
      }

      double theta = Math.atan2(frameLine.getDirection().getY(), frameLine.getDirection().getX());

      // It is scaled such that the line is being shifted by one cell or minDistanceFromLine depending on which one is the greatest.
      double cellXSize = occupancyGrid.getCellXSize();
      double cellYSize = occupancyGrid.getCellYSize();
      double distanceToMoveAwayFromLine = Math.max(minDistanceFromLine, Math.abs(cellXSize * Math.cos(theta) + cellYSize * Math.sin(theta)));
      shiftingVectorX *= distanceToMoveAwayFromLine;
      shiftingVectorY *= distanceToMoveAwayFromLine;

      // The point of the shiftedLine is shifted using the shiftingVector.
      double linePointX = frameLine.getPointX() + shiftingVectorX;
      double linePointY = frameLine.getPointY() + shiftingVectorY;
      double lineDirectionX = frameLine.getDirectionX();
      double lineDirectionY = frameLine.getDirectionY();

      int numberOfCellsActivatedOnSideToLookAt = 0;
      List<OccupancyGridCell> activeCells = occupancyGrid.getAllActiveCells();
      for (int i = 0; i < activeCells.size(); i++)
      {
         OccupancyGridCell cell = activeCells.get(i);
         double cellX = occupancyGrid.getXLocation(cell.getXIndex());
         double cellY = occupancyGrid.getXLocation(cell.getYIndex());

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(cellX, cellY, linePointX, linePointY, lineDirectionX, lineDirectionY, sideToLookAt == RobotSide.LEFT)
             && cell.getIsOccupied())
         {
            numberOfCellsActivatedOnSideToLookAt++;
         }
      }

      return numberOfCellsActivatedOnSideToLookAt;
   }
}
