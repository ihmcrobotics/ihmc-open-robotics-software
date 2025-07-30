package us.ihmc.perception.globalHeightMap;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class GlobalLattice
{
   public static final double latticeWidth = 5.0;

   public static int toIndex(double value)
   {
      return (int) (Math.floor(value / latticeWidth));
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

   public static int hashCodeOfTileIndices(double xIndex, double yIndex)
   {
      int ix = (int) Math.floor(xIndex);
      int iy = (int) Math.floor(yIndex);

      return 13 * ix + 17 * iy;
   }
}
