package us.ihmc.sensorProcessing.globalHeightMap;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class GlobalLattice
{
   public static final double latticeWidth = 5.0;

   public static int toIndex(double value)
   {
      return (int) (Math.round(value / latticeWidth));
   }

   public static double toPosition(int index)
   {
      return index * latticeWidth;
   }

   public static int hashCodeOfTile(Point2DReadOnly cellPosition)
   {
      return hashCodeOfTilePositions(cellPosition.getX(), cellPosition.getY());
   }

   public static int hashCodeOfTilePositions(double xPosition, double yPosition)
   {
      int x = toIndex(xPosition);
      int y = toIndex(yPosition);

      return hashCodeOfTileIndices(x, y);
   }

   public static int hashCodeOfTileIndices(int xIndex, int yIndex)
   {
      return 13 * xIndex + 17 * yIndex;
   }
}
