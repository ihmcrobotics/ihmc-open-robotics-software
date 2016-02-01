package us.ihmc.robotics.trajectories;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class LinearInterpolater
{
   private final double[] xPoints;
   private final double[] yPoints;
   private final int sizeOfVector;


   /**
    * xPoints must be in increasing order. In many cases x is the time of your trajectory.
    * @param xPoints double[].
    * @param yPoints double[]
    */
   public LinearInterpolater(double[] xPoints, double[] yPoints)
   {
      this.xPoints = xPoints;
      this.yPoints = yPoints;

      if (xPoints.length < 2)
      {
//       int foo = 0;
         throw new RuntimeException("LinearInterpolater: xPoints must have at least 2 points, length=" + xPoints.length);
      }

      if (!areXPointsInIncreasingOrder(xPoints))
      {
         System.err.println(Arrays.toString(xPoints));

         throw new RuntimeException("LinearInterpolater: xPoints must be in increasing order");
      }

      if (xPoints.length != yPoints.length)
      {
         throw new RuntimeException("LinearInterpolater: xPoints and yPoints must be equal length");
      }

      sizeOfVector = xPoints.length;
   }

   public LinearInterpolater(ArrayList<Double> xPointsArrayList, ArrayList<Double> yPointsArrayList) throws Exception
   {
      if (xPointsArrayList.size() != yPointsArrayList.size())
         throw new RuntimeException("array lists must be equal size");

      if (xPointsArrayList.size() < 2)
      {
//       int foo = 0;
         throw new Exception("LinearInterpolater: xPoints must have at least 2 points, length=" + xPointsArrayList.size());
      }


      if ((xPointsArrayList.get(1) - xPointsArrayList.get(0)) < 0.0)
      {
         Collections.reverse(xPointsArrayList);
         Collections.reverse(yPointsArrayList);
      }

      xPoints = new double[xPointsArrayList.size()];
      yPoints = new double[xPointsArrayList.size()];

      for (int i = 0; i < xPointsArrayList.size(); i++)
      {
         xPoints[i] = xPointsArrayList.get(i);
         yPoints[i] = yPointsArrayList.get(i);
      }

      if (!areXPointsInIncreasingOrder(xPoints))
      {
         System.err.println(Arrays.toString(xPoints));

         throw new Exception("LinearInterpolater: xPoints must be in increasing order");
      }

      sizeOfVector = xPoints.length;
   }

   // this will return the interpolated index
   public LinearInterpolater(ArrayList<Double> xPointsArrayList) throws Exception
   {
      if (xPointsArrayList.size() < 2)
      {
//       int foo = 0;
         throw new Exception("LinearInterpolater: xPoints must have at least 2 points, length=" + xPointsArrayList.size());
      }


      xPoints = new double[xPointsArrayList.size()];
      yPoints = new double[xPointsArrayList.size()];

      for (int i = 0; i < xPointsArrayList.size(); i++)
      {
         xPoints[i] = xPointsArrayList.get(i);
         yPoints[i] = i;
      }

      if (!areXPointsInIncreasingOrder(xPoints))
      {
         System.err.println(Arrays.toString(xPoints));

         throw new Exception("LinearInterpolater: xPoints must be in increasing order");
      }

      sizeOfVector = xPoints.length;
   }

   private int upperBoundIndex;
   private int lowerBoundIndex;

   public double getPoint(double xPointValue)
   {
      lowerBoundIndex = 0;
      upperBoundIndex = sizeOfVector - 1;

      boolean foundPoint = false;
      while (!foundPoint)
      {
         foundPoint = updateBounds(xPointValue);
      }

      if (upperBoundIndex == lowerBoundIndex)
      {
         return yPoints[upperBoundIndex];
      }

      else
      {
         return interpolation(xPointValue);
      }
   }


   public ArrayList<Double> getXpointsCopy()
   {
      ArrayList<Double> ret = new ArrayList<Double>();

      for (Double value : xPoints)
      {
         ret.add(value);
      }

      return ret;
   }

   public ArrayList<Double> getYpointsCopy()
   {
      ArrayList<Double> ret = new ArrayList<Double>();

      for (Double value : yPoints)
      {
         ret.add(value);
      }

      return ret;
   }




   /**
    * interpolation
    *
    * @param xPointValue double
    * @return double
    */
   private double interpolation(double xPointValue)
   {
      if ((upperBoundIndex - lowerBoundIndex) > 1)
      {
         throw new RuntimeException("LinearInterpolater: did not set the bounds properly");
      }

      double xMax = xPoints[upperBoundIndex];
      double xMin = xPoints[lowerBoundIndex];

      double yMax = yPoints[upperBoundIndex];
      double yMin = yPoints[lowerBoundIndex];

      if (xPointValue > xMax)
         return yMax;
      if (xPointValue < xMin)
         return yMin;

      double yDifference = yMax - yMin;
      double xDifference = xMax - xMin;

      double deltaX = xPointValue - xMin;
      double deltaY = deltaX * yDifference / xDifference;

      return yMin + deltaY;
   }

   /**
    * updateBounds
    *
    * @param xPointValue double
    * @return boolean
    */
   private boolean updateBounds(double xPointValue)
   {
      if ((upperBoundIndex - lowerBoundIndex) <= 1)
      {
         return true;
      }

      int middlePointIndex = (int) (((upperBoundIndex + lowerBoundIndex)) / 2.0);
      double xPointAtMiddle = xPoints[middlePointIndex];

      if (xPointValue > xPointAtMiddle)
      {
         lowerBoundIndex = middlePointIndex;

         if ((upperBoundIndex - lowerBoundIndex) <= 1)
         {
            return true;
         }

      }
      else if (xPointValue < xPointAtMiddle)
      {
         upperBoundIndex = middlePointIndex;

         if ((upperBoundIndex - lowerBoundIndex) <= 1)
         {
            return true;
         }
      }
      else
      {
         upperBoundIndex = middlePointIndex;
         lowerBoundIndex = middlePointIndex;

         return true;
      }

      return false;
   }

   /**
    * areXPointsInIncreasingOrder
    *
    * @param xPoints double[]
    * @return boolean
    */
   private boolean areXPointsInIncreasingOrder(double[] points)
   {
      for (int i = 1; i < points.length; i++)
      {
         if (points[i] <= points[i - 1])
         {
            return false;
         }
      }

      return true;
   }

   public double getMaxX()
   {
      return xPoints[xPoints.length - 1];
   }

   public double getMinX()
   {
      return xPoints[0];
   }


   public static void main(String[] args)
   {
      int numberOfPoints = 4;
      double[] xTestPoints = new double[numberOfPoints];
      double[] yTestPoints = new double[numberOfPoints];

      @SuppressWarnings("unused")
      double xStart = 0.0;
      @SuppressWarnings("unused")
      double deltaX = 1.0;

//    for (int i=0; i< numberOfPoints; i++)
//    {
//       xTestPoints[i] = xStart + deltaX * i;
//       yTestPoints[i] =  xTestPoints[i] * xTestPoints[i];
//    }
//
//    xTestPoints[5] = xTestPoints[4];

      xTestPoints[0] = 0.0;
      xTestPoints[1] = 0.5;
      xTestPoints[2] = 1.0;
      xTestPoints[3] = 2.0;

      yTestPoints[0] = 0.0;
      yTestPoints[1] = 1.0;
      yTestPoints[2] = 2.0;
      yTestPoints[3] = 3.0;


      LinearInterpolater linInterp = null;
      try
      {
         linInterp = new LinearInterpolater(xTestPoints, yTestPoints);
      }
      catch (Exception ex)
      {
         System.err.println(ex);
      }


//    for (int i=0; i< numberOfPoints*20; i++)
//    {
//       double xTest = xStart + deltaX * i - 5.0;
//       System.out.println("x=" + xTest + ", y=" + linInterp.getPoint(xTest));
//    }

      for (double x = -1.0; x <= 4.0; x = x + 0.5)
      {
         System.out.println("x=" + x + ", y=" + linInterp.getPoint(x));
      }
   }
}
