package us.ihmc.robotics.dataStructures;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.InclusionFunction;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

public class DoubleHashHeightMap implements HeightMapWithPoints
{
   private HashMap<Integer, LinkedHashMap<Integer, Double>> rows;
   private final double gridSize;

   public DoubleHashHeightMap(double gridSize)
   {
      this.gridSize = gridSize;
      rows = new LinkedHashMap<Integer, LinkedHashMap<Integer, Double>>();
   }

   public double getHeightAtPoint(double x, double y)
   {
      return heightAtPoint(index(x), index(y));
   }

   public boolean containsPoint(double x, double y)
   {
      return containsPoint(index(x), index(y));
   }

   private boolean containsPoint(int x, int y)
   {
      if (!rows.containsKey(x))
         return false;
      HashMap<Integer, Double> row = rows.get(x);
      if (!row.containsKey(y))
         return false;
      Double height = row.get(y);
      if (!MathTools.isFinite(height))
         return false;

      return true;

   }

   private double heightAtPoint(int x, int y)
   {
      if (!rows.containsKey(x))
         return Double.NaN;
      HashMap<Integer, Double> row = rows.get(x);
      if (!row.containsKey(y))
         return Double.NaN;
      Double height = row.get(y);

      return height;

   }

   private boolean addPoint(int xIndex, int yIndex, double z)
   {

      synchronized (rows)
      {
         if (MathTools.isFinite(z))
         {
            if (!rows.containsKey(xIndex))
            {
               rows.put(xIndex, new LinkedHashMap<Integer, Double>());
            }


            HashMap<Integer, Double> xRow = rows.get(xIndex);
            xRow.put(yIndex, z);
         }
      }

      return true;
   }

   public boolean addPoint(double x, double y, double z)
   {
      return addPoint(index(x), index(y), z);
   }

   private int index(double coordinate)
   {
      double quotient = coordinate / gridSize;
      int closestIndex = (int) Math.round(quotient);

      return closestIndex;
   }

   public double gridSize()
   {
      return this.gridSize;
   }

   private List<Point3d> getAllPointsWithin(int xMin, int xMax, int yMin, int yMax)
   {
      ArrayList<Point3d> points = new ArrayList<Point3d>();
      if ((xMax >= xMin) && (yMax >= yMin))
      {
         if (rows.values().size() < (xMax - xMin+1))
         {
            iterateOverKeysToPackAllRowsWithin(xMin, xMax, yMin, yMax, points);
         }
         else
         {
            iterateOverValuesToPackAllRowsWithin(xMin, xMax, yMin, yMax, points);
         }
      }

      return points;
   }

   public void clear()
   {
      synchronized (rows)
      {
         rows.clear();
      }
   }

   private void iterateOverKeysToPackAllRowsWithin(int xMin, int xMax, int yMin, int yMax, ArrayList<Point3d> points)
   {
      synchronized (rows)
      {
         for (int x : rows.keySet())
         {
            if ((xMin <= x) && (xMax >= x))
            {
               LinkedHashMap<Integer, Double> row = new LinkedHashMap<Integer, Double>(rows.get(x));
               packAllPointsInRow(yMin, yMax, points, x, row);
            }
         }
      }
   }

   private void iterateOverValuesToPackAllRowsWithin(int xMin, int xMax, int yMin, int yMax, ArrayList<Point3d> points)
   {
      synchronized (rows)
      {
         for (int x = xMin; x <= xMax; x++)
         {
            if (rows.containsKey(x))
            {
               HashMap<Integer, Double> row = rows.get(x);
               packAllPointsInRow(yMin, yMax, points, x, row);
            }
         }
      }
   }

   private void packAllPointsInRow(int yMin, int yMax, ArrayList<Point3d> points, int x, HashMap<Integer, Double> row)
   {
      if (row.values().size() < (yMax - yMin+1))
      {
         iterateOverKeysetToPackAllPoints(yMin, yMax, points, x, row);
      }
      else
      {
         iterateOverColumnsToPackAllPoints(yMin, yMax, points, x, row);
      }
   }

   private void iterateOverColumnsToPackAllPoints(int yMin, int yMax, ArrayList<Point3d> points, int x, HashMap<Integer, Double> row)
   {
      for (int y = yMin; y <= yMax; y++)
      {
         if (row.containsKey(y))
         {
            points.add(new Point3d(x * gridSize, y * gridSize, row.get(y)));
         }
      }
   }

   private void iterateOverKeysetToPackAllPoints(int yMin, int yMax, ArrayList<Point3d> points, int x, HashMap<Integer, Double> row)
   {
      for (int y : row.keySet())
      {
         if ((yMin <= y) && (yMax >= y))
         {
            points.add(new Point3d(x * gridSize, y * gridSize, row.get(y)));
         }
      }
   }

   public List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent)
   {
      return getAllPointsWithinAreaMinMax(xCenter - (xExtent * 0.5), xCenter + (xExtent * 0.5), yCenter - (yExtent * 0.5), yCenter + (yExtent * 0.5));
   }

   private List<Point3d> getAllPointsWithinAreaMinMax(double xMin, double xMax, double yMin, double yMax)
   {
      return getAllPointsWithin(index(xMin), index(xMax), index(yMin), index(yMax));
   }

   public List<Point3d> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent,
         InclusionFunction<Point3d> maskFunctionAboutCenter)
   {
      List<Point3d> allPointsInArea = getAllPointsWithinArea(xCenter, yCenter, xExtent, yExtent);
      List<Point3d> pointsToReturn = new ArrayList<Point3d>();
      for (Point3d point : allPointsInArea)
      {
           if (maskFunctionAboutCenter.isIncluded(point))
            pointsToReturn.add(point);
      }
      return pointsToReturn;
   }


}
