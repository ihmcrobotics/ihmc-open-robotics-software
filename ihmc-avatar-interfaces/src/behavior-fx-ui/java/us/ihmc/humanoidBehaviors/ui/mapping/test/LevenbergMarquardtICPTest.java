package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple2D.Point2D;

public class LevenbergMarquardtICPTest
{
   private XYPlaneDrawer drawer;
   private JFrame frame;

   private List<Point2D> fullModel = new ArrayList<>();
   private List<Point2D> data1 = new ArrayList<>();
   private List<Point2D> data2 = new ArrayList<>();

   private void setupPointCloud()
   {
      drawer = new XYPlaneDrawer(10.0, -10.0, 10.0, -10.0);
      frame = new JFrame("2D_LM_ICP_TEST");

      frame.setPreferredSize(drawer.getDimension());
      frame.setLocation(200, 100);

      double innerCircleLong = 2.0;
      double innerCircleShort = 1.0;
      double outterCircleLong = 4.0;
      double outterCircleShort = 3.0;
      fullModel.addAll(generatePointsOnEllipsoid(50, Math.toRadians(90.0), Math.toRadians(359.9), innerCircleLong, innerCircleShort));
      fullModel.addAll(generatePointsOnEllipsoid(70, Math.toRadians(90.0), Math.toRadians(359.9), outterCircleLong, outterCircleShort));
      fullModel.addAll(generatePointsOnLine(10, innerCircleShort, outterCircleShort, 0.0, true));
      fullModel.addAll(generatePointsOnLine(10, innerCircleLong, outterCircleLong, 0.0, false));

      data1.addAll(generatePointsOnEllipsoid(35, Math.toRadians(90.0), Math.toRadians(200.0), innerCircleLong, innerCircleShort));
      data1.addAll(generatePointsOnEllipsoid(50, Math.toRadians(90.0), Math.toRadians(320.0), outterCircleLong, outterCircleShort));
      data1.addAll(generatePointsOnLine(10, innerCircleShort, outterCircleShort, 0.0, true));

      data2.addAll(generatePointsOnEllipsoid(50, Math.toRadians(130.0), Math.toRadians(359.9), innerCircleLong, innerCircleShort));
      data2.addAll(generatePointsOnEllipsoid(60, Math.toRadians(120.0), Math.toRadians(359.9), outterCircleLong, outterCircleShort));
      data2.addAll(generatePointsOnLine(10, innerCircleLong, outterCircleLong, 0.0, false));
   }

   @Test
   public void testVisualization()
   {
      setupPointCloud();

      transformPointCloud(data1, 1.0, 2.0, Math.toRadians(30.0));
      transformPointCloud(data2, -1.0, -2.0, Math.toRadians(-90));

      drawer.addPointCloud(fullModel, Color.black, false);
      drawer.addPointCloud(data1, Color.red, false);
      drawer.addPointCloud(data2, Color.green, false);

      frame.add(drawer);
      frame.pack();
      frame.setVisible(true);

      ThreadTools.sleepForever();
   }

   @Test
   public void testFindingClosestPointWithFullModel()
   {
      setupPointCloud();

      transformPointCloud(data1, 0.3, 0.5, Math.toRadians(10.0));

      drawer.addPointCloud(fullModel, Color.black, false);
      drawer.addPointCloud(data1, Color.red, false);

      double outlierDistance = 0.2;
      for (int i = 0; i < data1.size(); i++)
      {
         double distance = computeClosestDistance(data1.get(i), fullModel);
         if (distance < outlierDistance)
            drawer.addPoint(data1.get(i), Color.red, true);
         else
            drawer.addPoint(data1.get(i), Color.red, false);
      }

      frame.add(drawer);
      frame.pack();
      frame.setVisible(true);

      ThreadTools.sleepForever();
   }

   @Test
   public void testCorrespondenceWeightFactor()
   {

   }

   @Test
   public void testErrorFunctionDerivation()
   {

   }

   @Test
   public void testJacobian()
   {

   }

   @Test
   public void testIteration()
   {

   }

   private double computeClosestDistance(Point2D point, List<Point2D> pointCloud)
   {
      double minDistance = Double.MAX_VALUE;
      double distance = 0.0;

      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point2D modelPoint = pointCloud.get(i);

         distance = point.distance(modelPoint);
         if (distance < minDistance)
         {
            minDistance = distance;
         }
      }

      return minDistance;
   }

   private void transformPointCloud(List<Point2D> pointCloud, double translationX, double translationY, double theta)
   {
      double sin = Math.sin(theta);
      double cos = Math.cos(theta);
      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point2D point = pointCloud.get(i);

         double transformedX = cos * point.getX() - sin * point.getY() + translationX;
         double transformedY = sin * point.getX() + cos * point.getY() + translationY;

         point.set(transformedX, transformedY);
      }
   }

   private List<Point2D> generatePointsOnLine(int numberOfPoints, double start, double end, double fix, boolean isXFixed)
   {
      List<Point2D> points = new ArrayList<Point2D>();

      for (int i = 0; i < numberOfPoints; i++)
      {
         double x, y = 0;
         if (isXFixed)
         {
            x = fix;
            y = start + (end - start) * i / (numberOfPoints - 1);
         }
         else
         {
            x = start + (end - start) * i / (numberOfPoints - 1);
            y = fix;
         }
         points.add(new Point2D(x, y));
      }

      return points;
   }

   private List<Point2D> generatePointsOnEllipsoid(int numberOfPoints, double thetaStart, double thetaEnd, double xRadius, double yRadius)
   {
      List<Point2D> points = new ArrayList<Point2D>();

      double radius, theta, sin, cos = 0;

      for (int i = 0; i < numberOfPoints; i++)
      {
         theta = thetaStart + (thetaEnd - thetaStart) * i / (numberOfPoints - 1);
         sin = Math.sin(theta);
         cos = Math.cos(theta);

         radius = Math.sqrt((xRadius * xRadius * yRadius * yRadius) / (yRadius * yRadius - yRadius * yRadius * sin * sin + xRadius * xRadius * sin * sin));

         double x = radius * cos;
         double y = radius * sin;
         points.add(new Point2D(x, y));
      }

      return points;
   }

   private static class XYPlaneDrawer extends JPanel
   {
      private static final long serialVersionUID = 1L;
      private static final int scale = 40;
      private final List<Point2D> pointCloud;
      private final List<Color> pointCloudColors;
      private final List<Boolean> fillers;

      private final double xUpper;
      private final double yUpper;
      private final int sizeU;
      private final int sizeV;

      XYPlaneDrawer(double xUpper, double xLower, double yUpper, double yLower)
      {
         pointCloud = new ArrayList<>();
         pointCloudColors = new ArrayList<>();
         fillers = new ArrayList<>();
         this.xUpper = xUpper;
         this.yUpper = yUpper;
         sizeU = (int) Math.round((-yLower + yUpper) * scale);
         sizeV = (int) Math.round((-xLower + xUpper) * scale);
      }

      public void addPoint(Point2D point, Color color, boolean fill)
      {
         pointCloud.add(point);
         pointCloudColors.add(color);
         fillers.add(fill);
      }

      public void addPointCloud(List<Point2D> points, Color color, boolean fill)
      {
         pointCloud.addAll(points);
         for (int i = 0; i < points.size(); i++)
         {
            pointCloudColors.add(color);
            fillers.add(fill);
         }
      }

      public void paint(Graphics g)
      {
         super.paint(g);
         g.setColor(Color.red);
         g.drawLine(x2u(1.5), y2v(0.0), x2u(0.0), y2v(0.0));
         g.setColor(Color.green);
         g.drawLine(x2u(0.0), y2v(1.5), x2u(0.0), y2v(0.0));

         g.setColor(Color.black);
         pointFill(g, 0, 0, 4);

         for (int i = 0; i < pointCloud.size(); i++)
         {
            g.setColor(pointCloudColors.get(i));
            Point2D point = pointCloud.get(i);
            if (fillers.get(i))
               pointFill(g, point.getX(), point.getY(), 4);
            else
               point(g, point.getX(), point.getY(), 4);
         }
      }

      public void point(Graphics g, double px, double py, int size)
      {
         int diameter = size;
         g.drawOval(x2u(px) - diameter / 2, y2v(py) - diameter / 2, diameter, diameter);
      }

      public void pointFill(Graphics g, double px, double py, int size)
      {
         int diameter = size;
         g.fillOval(x2u(px) - diameter / 2, y2v(py) - diameter / 2, diameter, diameter);
      }

      public int x2u(double px)
      {
         return (int) Math.round(((px) + xUpper) * scale);
      }

      public int y2v(double py)
      {
         return (int) Math.round(((-py) + yUpper) * scale);
      }

      public Dimension getDimension()
      {
         return new Dimension(sizeU, sizeV);
      }
   }
}
