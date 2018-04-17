package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;

/*
 * TODO This class should be cleaned up really good, tested, and renamed LinearRegression3D, so it
 * can eventually move to a lower level library.
 * Check out IncrementalCovariance3D as reference.
 */
public class LinearRegression3D
{
   private List<Point3D> points;
   private double beta1;
   private double beta0;

   public LinearRegression3D(List<Point3D> points)
   {
      this.points = points;
   }

   public void calculateRegression()
   {
      int MAXN = 1000;
      int n = 0;
      double[] x = new double[MAXN];
      double[] y = new double[MAXN];

      // first pass: read in data, compute xbar and ybar
      double sumx = 0.0, sumy = 0.0, sumx2 = 0.0;

      for (int i = 0; i < points.size(); i++)
      {
         x[n] = points.get(n).getX();
         y[n] = points.get(n).getY();
         sumx += x[n];
         sumx2 += x[n] * x[n];
         sumy += y[n];
         n++;
      }

      double xbar = sumx / n;
      double ybar = sumy / n;

      // second pass: compute summary statistics
      double xxbar = 0.0, yybar = 0.0, xybar = 0.0;
      for (int i = 0; i < n; i++)
      {
         xxbar += (x[i] - xbar) * (x[i] - xbar);
         yybar += (y[i] - ybar) * (y[i] - ybar);
         xybar += (x[i] - xbar) * (y[i] - ybar);
      }
      beta1 = xybar / xxbar;
      beta0 = ybar - beta1 * xbar;

      // print results
      //      System.out.println("y   = " + beta1 + " * x + " + beta0);

      // analyze results
      int df = n - 2;
      double rss = 0.0; // residual sum of squares
      double ssr = 0.0; // regression sum of squares
      for (int i = 0; i < n; i++)
      {
         double fit = beta1 * x[i] + beta0;
         rss += (fit - y[i]) * (fit - y[i]);
         ssr += (fit - ybar) * (fit - ybar);
      }
      double R2 = ssr / yybar;
      double svar = rss / df;
      double svar1 = svar / xxbar;
      double svar0 = svar / n + xbar * xbar * svar1;

      //      System.out.println("R^2                 = " + R2);
      //      System.out.println("std error of beta_1 = " + Math.sqrt(svar1));
      //      System.out.println("std error of beta_0 = " + Math.sqrt(svar0));
      //      svar0 = svar * sumx2 / (n * xxbar);
      //      System.out.println("std error of beta_0 = " + Math.sqrt(svar0));
      //
      //      System.out.println("SSTO = " + yybar);
      //      System.out.println("SSE  = " + rss);
      //      System.out.println("SSR  = " + ssr);
   }

   public double getYFromX(double x)
   {
      return beta1 * x + beta0;
   }

   public Point3D[] getTheTwoPointsFurthestApart()
   {
      double maxDist = 0.0;
      Point3D pointA = new Point3D();
      Point3D pointB = new Point3D();

      for (Point3D observer : points)
      {
         for (Point3D target : points)
         {
            double distance = observer.distance(target);

            if (distance > maxDist)
            {
               maxDist = distance;
               pointA = observer;
               pointB = target;
            }
         }
      }
      return new Point3D[] {pointA, pointB};
   }
}