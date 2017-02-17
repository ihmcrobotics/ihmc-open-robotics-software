package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.util.ShapeUtilities;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Point2D;

public class InPlaceConvexHullCalculator2dTest
{
   private static final boolean DEBUG = true;
   Random rand = new Random(100l);

	@ContinuousIntegrationTest(estimatedDuration = 2.6)
	@Test(timeout = 30000)
   public void testRandomV1()
   {
      int iterations = 5000;
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      ArrayList<Point2D> inPlaceHull = new ArrayList<Point2D>();

      long optClock = 0;
      long inPlaceClock = 0;
      for (; iterations > 0; iterations--)
      {
         points.clear();
         inPlaceHull.clear();

         for (int i = 0; i < 150; i++)
         {
            double x = rand.nextDouble();
            double y = rand.nextDouble();
            points.add(new Point2D(x, y));
            inPlaceHull.add(new Point2D(x, y));
         }

         optClock -= System.nanoTime();
         List<Point2D> hull = ConvexHullCalculator2d.getConvexHullCopy(points);
         optClock += System.nanoTime();

         inPlaceClock -= System.nanoTime();
         InPlaceConvexHullCalculator2d.inPlaceGiftWrapConvexHull2d(inPlaceHull);
         inPlaceClock += System.nanoTime();

         boolean sizesMatch = hull.size() == inPlaceHull.size();
         boolean convexAndClockwise1 = ConvexHullCalculator2d.isConvexAndClockwise(inPlaceHull);
         boolean convexAndClockwise2 = InPlaceConvexHullCalculator2d.isConvexAndClockwise(inPlaceHull);

         if (!sizesMatch || !convexAndClockwise1 || !convexAndClockwise2)
            printForDebug(points, inPlaceHull.size(), hull, iterations);

         assertTrue(sizesMatch);
         assertTrue(convexAndClockwise1);
         assertTrue(convexAndClockwise2);
         //assertTrue(hull.size() == inPlaceHull.size());
      }
      System.out.println(optClock / 1000000 + " " + inPlaceClock / 1000000);
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.1)
	@Test(timeout = 30000)
   public void testRandomV2()
   {
      int iterations = 5000;
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      ArrayList<Point2D> inPlaceHull = new ArrayList<Point2D>();

      long optClock = 0;
      long inPlaceClock = 0;
      for (; iterations > 0; iterations--)
      {
         points.clear();
         inPlaceHull.clear();

         for (int i = 0; i < 150; i++)
         {
            double x = rand.nextDouble();
            double y = rand.nextDouble();
            points.add(new Point2D(x, y));
            inPlaceHull.add(new Point2D(x, y));
         }

         optClock -= System.nanoTime();
         List<Point2D> hull = ConvexHullCalculator2d.getConvexHullCopy(points);
         optClock += System.nanoTime();

         inPlaceClock -= System.nanoTime();
         int originalSize = inPlaceHull.size();
         int inPlaceHullNewSize = InPlaceConvexHullCalculator2d.inPlaceGiftWrapConvexHull2d(inPlaceHull, originalSize);
         inPlaceClock += System.nanoTime();

         boolean sizesMatch = hull.size() == inPlaceHullNewSize;
         boolean convexAndClockwise1 = ConvexHullCalculator2d.isConvexAndClockwise(inPlaceHull, inPlaceHullNewSize);
         boolean convexAndClockwise2 = InPlaceConvexHullCalculator2d.isConvexAndClockwise(inPlaceHull, inPlaceHullNewSize);

         if (!sizesMatch || !convexAndClockwise1 || !convexAndClockwise2)
            printForDebug(inPlaceHull, inPlaceHullNewSize, hull, iterations);

         assertTrue(sizesMatch);
         assertTrue(convexAndClockwise1);
         assertTrue(convexAndClockwise2);
         //assertTrue(hull.size() == inPlaceHull.size());
      }
      System.out.println(optClock / 1000000 + " " + inPlaceClock / 1000000);
   }

   private void printForDebug(ArrayList<Point2D> points, int newSize, List<Point2D> hull, int iterations)
   {
      if (DEBUG)
      {
         System.out.println("Iteration: " + iterations);
         int maxSize = Math.max(hull.size(), newSize);
         for (int i = 0; i < maxSize; i++)
         {
            String hullPoint = i >= hull.size() ? "null" : hull.get(i).toString();
            String pointsString = i >= newSize ? "null" : points.get(i).toString();
            System.out.println("NewStuff: " + pointsString + ", Hull: " + hullPoint);
         }
         System.out.println();
         plot(points);
         plot(hull);
      }
   }

   public static void plot(List<Point2D> points)
   {
      double[][] data = new double[2][points.size()];
      for (int i = 0; i < points.size(); i++)
      {
         data[0][i] = points.get(i).getX();
         data[1][i] = points.get(i).getY();
      }

      JFreeChart jfreechart = ChartFactory
            .createScatterPlot("Scatter Plot Demo", "X", "Y", sampleFromData(data), PlotOrientation.VERTICAL, true, true, false);
      XYPlot xyPlot = (XYPlot) jfreechart.getPlot();
      xyPlot.setDomainCrosshairVisible(true);
      xyPlot.setRangeCrosshairVisible(true);
      XYItemRenderer renderer = xyPlot.getRenderer();
      renderer.setSeriesShape(0, ShapeUtilities.createDiagonalCross(3, 1));

      JFrame jf = new JFrame();
      jf.setContentPane(new ChartPanel(jfreechart));
      jf.setVisible(true);

   }

   private static XYDataset sampleFromData(double[][] data)
   {
      XYSeriesCollection xySeriesCollection = new XYSeriesCollection();
      XYSeries series = new XYSeries("Random");
      for (int i = 0; i < data[0].length; i++)
      {
         series.add(data[0][i], data[1][i]);
      }
      xySeriesCollection.addSeries(series);
      return xySeriesCollection;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConvexCounterclockwise()
   {
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      points.add(new Point2D(-1, -1));
      points.add(new Point2D(1, -1));
      points.add(new Point2D(1, 1));
      points.add(new Point2D(-1, 1));

      List<Point2D> hull = ConvexHullCalculator2d.getConvexHullCopy(points);
      int originalSize = points.size();
      int newSize = InPlaceConvexHullCalculator2d.inPlaceGiftWrapConvexHull2d(points, originalSize);
      
      assertTrue(hull.size() == newSize);
      assertTrue(ConvexHullCalculator2d.isConvexAndClockwise(points, newSize));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFirstPointSelection()
   {
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      points.add(new Point2D(1, -1));
      points.add(new Point2D(-1, -1));
      points.add(new Point2D(1, 1));
      points.add(new Point2D(-1, 1));

      List<Point2D> hull = ConvexHullCalculator2d.getConvexHullCopy(points);
      int originalSize = points.size();
      int newSize = InPlaceConvexHullCalculator2d.inPlaceGiftWrapConvexHull2d(points, originalSize);

      assertTrue(hull.size() == originalSize);
      assertTrue(ConvexHullCalculator2d.isConvexAndClockwise(points, newSize));
   }

   public static void main(String[] args)
   {
      InPlaceConvexHullCalculator2dTest test = new InPlaceConvexHullCalculator2dTest();
      test.testRandomV1();
   }
}
