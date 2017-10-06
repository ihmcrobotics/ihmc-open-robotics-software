package us.ihmc.robotics.trajectories;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;

public class CubicSplineCurveGenerator implements CurveGenerator
{
   private double[] yValues;    // y=f(x) tabulated function
   private double[] xValues;    // x in tabulated function f(x)   private  double xMin, xMax;
   private double xMin, xMax;
   private double firstDerivativeAtStart = Double.NaN;
   private double firstDerivativeAtEnd = Double.NaN;
   private boolean derivCalculated = false;
   private double[] d2ydx2 = null;    // second derivatives of y


   public CubicSplineCurveGenerator(Point2D[] points)
   {
      init(points);
   }

   public CubicSplineCurveGenerator(ArrayList<Point2D> points)
   {
      Point2D[] arrayPoints = new Point2D[points.size()];
      points.toArray(arrayPoints);

      init(arrayPoints);
   }

   public CubicSplineCurveGenerator(double[] points)
   {
      Point2D[] arrayOfPoints = new Point2D[points.length];

      double numberOfPoints = points.length;

      for (int i = 0; i < points.length; i++)
      {
         arrayOfPoints[i] = new Point2D();
         arrayOfPoints[i].setY(points[i]);
         arrayOfPoints[i].setX((i + 1.0) / numberOfPoints);
      }

      init(arrayOfPoints);
   }




   private void init(Point2D[] points)
   {
      this.xValues = new double[points.length];
      this.yValues = new double[points.length];

      double xMinTemp = Double.POSITIVE_INFINITY;
      double xMaxTemp = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < points.length; i++)
      {
         this.xValues[i] = points[i].getX();
         this.yValues[i] = points[i].getY();

         xMinTemp = Math.min(xMinTemp, points[i].getX());
         xMaxTemp = Math.max(xMaxTemp, points[i].getX());

         // make sure that x is in increasing order
         if (i > 0)
         {
            if ((points[i].getX() - points[i - 1].getX()) <= 0.0)
               throw new RuntimeException("Points must be in increasing x order");
         }
      }

      xMin = xMinTemp;
      xMax = xMaxTemp;

//    yp1 = Double.NaN;
//    ypn = Double.NaN;
      derivCalculated = false;
   }


   // Enters the first derivatives of the cubic spline at
   // the first and last point of the tabulated data
   // Overrides a natural spline
   public void setStartAndEndDerivatives(double startFirstDerivative, double endFirstDerivative)
   {
      this.firstDerivativeAtStart = startFirstDerivative;
      this.firstDerivativeAtEnd = endFirstDerivative;

      this.derivCalculated = false;
   }

   // Resets a natural spline
   // Use above - this kept for backward compatibility
   public void resetFirstDerivatives()
   {
      this.firstDerivativeAtStart = Double.NaN;
      this.firstDerivativeAtEnd = Double.NaN;
   }

   // Calculates the second derivatives of the tabulated function
   // for use by the cubic spline interpolation method (.interpolate)
   // This method follows the procedure in Numerical Methods C language procedure for calculating second derivatives
   private void calcDeriv()
   {
      double p = 0.0D, qn = 0.0D, sig = 0.0D, un = 0.0D;
      double[] u = new double[xValues.length];
      d2ydx2 = new double[xValues.length];

//    if (this.yp1 != this.yp1)
      if (Double.isNaN(this.firstDerivativeAtStart))
      {
         d2ydx2[0] = u[0] = 0.0;
      }
      else
      {
         this.d2ydx2[0] = -0.5;
         u[0] = (3.0 / (this.xValues[1] - this.xValues[0]))
                * ((this.yValues[1] - this.yValues[0]) / (this.xValues[1] - this.xValues[0]) - this.firstDerivativeAtStart);
      }

      for (int i = 1; i <= this.xValues.length - 2; i++)
      {
         sig = (this.xValues[i] - this.xValues[i - 1]) / (this.xValues[i + 1] - this.xValues[i - 1]);
         p = sig * this.d2ydx2[i - 1] + 2.0;
         this.d2ydx2[i] = (sig - 1.0) / p;
         u[i] = (this.yValues[i + 1] - this.yValues[i]) / (this.xValues[i + 1] - this.xValues[i])
                - (this.yValues[i] - this.yValues[i - 1]) / (this.xValues[i] - this.xValues[i - 1]);
         u[i] = (6.0 * u[i] / (this.xValues[i + 1] - this.xValues[i - 1]) - sig * u[i - 1]) / p;
      }

//    if (this.ypn != this.ypn)
      if (Double.isNaN(this.firstDerivativeAtEnd))
      {
         qn = un = 0.0;
      }
      else
      {
         qn = 0.5;
         un = (3.0 / (this.xValues[xValues.length - 1] - this.xValues[this.xValues.length - 2]))
              * (this.firstDerivativeAtEnd
                 - (this.yValues[this.xValues.length - 1] - this.yValues[this.xValues.length - 2])
                   / (this.xValues[this.xValues.length - 1] - xValues[this.xValues.length - 2]));
      }

      this.d2ydx2[this.xValues.length - 1] = (un - qn * u[this.xValues.length - 2]) / (qn * this.d2ydx2[this.xValues.length - 2] + 1.0);

      for (int k = this.xValues.length - 2; k >= 0; k--)
      {
         this.d2ydx2[k] = this.d2ydx2[k] * this.d2ydx2[k + 1] + u[k];
      }

      this.derivCalculated = true;
   }

   public Point2D getPointGivenX(double xValue)
   {
      double yValue = interpolate(xValue);
      return new Point2D(xValue, yValue);

   }

   public double interpolate(double xValue)
   {
      if ((xValue < xMin) || (xValue > xMax))
      {
         throw new IllegalArgumentException("x (" + xValue + ") is outside the range of data points");
      }

      if (!this.derivCalculated)
         this.calcDeriv();

      double h = 0.0D, b = 0.0D, a = 0.0D, yValue = 0.0D;
      int k = 0;
      int klo = 0;
      int khi = this.xValues.length - 1;
      while (khi - klo > 1)
      {
         k = (khi + klo) >> 1;

         if (this.xValues[k] > xValue)
         {
            khi = k;
         }
         else
         {
            klo = k;
         }
      }

      h = this.xValues[khi] - this.xValues[klo];

      if (h == 0.0)
      {
         throw new IllegalArgumentException("Two values of x are identical: point " + klo + " (" + this.xValues[klo] + ") and point " + khi + " ("
                                            + this.xValues[khi] + ")");
      }
      else
      {
         a = (this.xValues[khi] - xValue) / h;
         b = (xValue - this.xValues[klo]) / h;
         yValue = a * this.yValues[klo] + b * this.yValues[khi] + ((a * a * a - a) * this.d2ydx2[klo] + (b * b * b - b) * this.d2ydx2[khi]) * (h * h) / 6.0;
      }
      return yValue;
   }

   public double getDerivative(double xValue)
   {
      return 0.0;
   }

   public double getXmin()
   {
      return xMin;
   }

   public double getXmax()
   {
      return xMax;
   }

   public Point2D[] getArrayOfPoints(int numberOfPointsToReturn)
   {
      if (numberOfPointsToReturn < 2)
         numberOfPointsToReturn = 2;

      double stepSize = (xMax - xMin) / ((double) numberOfPointsToReturn - 1);

      Point2D[] ret = new Point2D[numberOfPointsToReturn];
      double x = xMin;

      for (int i = 0; i < numberOfPointsToReturn; i++)
      {
         ret[i] = getPointGivenX(x);
         x += stepSize;
         if (x > xMax)
            x = xMax;
      }

      return ret;
   }

   public void setArrayOfPoints(Point2D[] newArrayOfPoints)
   {
      init(newArrayOfPoints);
   }

   public static void main(String[] args)
   {
//    Point2d point1 = new Point2d(0.0, 0.0);
//    Point2d point2 = new Point2d(0.7, 1.3);
//    Point2d point3 = new Point2d(1.8, 2.0);
//    Point2d point4 = new Point2d(2.3, 6.9);
//    Point2d point5 = new Point2d(2.6, 2.0);
//    Point2d point6 = new Point2d(3.0, 6.9);
//
//    ArrayList<Point2d> listOfPoints = new ArrayList<Point2d> ();
//    listOfPoints.add(point1);
//    listOfPoints.add(point2);
//    listOfPoints.add(point3);
////    listOfPoints.add(point4);
////    listOfPoints.add(point5);
////    listOfPoints.add(point6);
//
//
//
//    PointAndLinePlotter pointAndLinePlotter = new PointAndLinePlotter(listOfPoints, 8);
//
//    CubicSplineCurveGenerator curveGenerator = new CubicSplineCurveGenerator(listOfPoints);
//    Point2d[] splinePoints = curveGenerator.getArrayOfPoints(1000);
//
//    pointAndLinePlotter.addPoints(splinePoints, Color.CYAN, 4);
//
//
//
//    curveGenerator = new CubicSplineCurveGenerator(listOfPoints);
//    curveGenerator.setStartAndEndDerivatives(-10.0, Double.NaN);
//    splinePoints = curveGenerator.getArrayOfPoints(1000);
//
//    pointAndLinePlotter.addPoints(splinePoints, Color.GREEN, 4);

//    double[] deriv = MathTools.getFirstDerivatives(splinePoints);
//
//    JohnsUsefulFunctions.printArray(deriv, System.out);
//
//    System.out.println("Done");
//
//     double[] arrayPointsX = new double[splinePoints.length];
//     double[] arrayPointsY = new double[splinePoints.length];
//     for(int i = 0; i< splinePoints.length; i++)
//     {
//        arrayPointsX[i] = splinePoints[i].x;
//        arrayPointsY[i] = splinePoints[i].y;
//     }


//     double[] doublePoints = new double[]{0.0, 4.0};
//     PointAndLinePlotter pointAndLinePlotter = new PointAndLinePlotter(doublePoints, 8);




//     PlotGraph plotGraph = new PlotGraph(arrayPointsX, arrayPointsY);
//     plotGraph.plot();



   }

}
