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

   public static int hashCodeOfCell(Point2DReadOnly cellPosition)
   {
      return hashCodeOfCell(cellPosition.getX(), cellPosition.getY());
   }

   public static int hashCodeOfCell(double xPosition, double yPosition)
   {
      int x = toIndex(xPosition);
      int y = toIndex(yPosition);

      return hashCodeOfCell(x, y);
   }

   public static int hashCodeOfCell(int x, int y)
   {
      return 13 * x + 17 * y;
   }
}
