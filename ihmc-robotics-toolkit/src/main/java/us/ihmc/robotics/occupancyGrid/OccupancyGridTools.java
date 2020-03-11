package us.ihmc.robotics.occupancyGrid;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;

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
}
