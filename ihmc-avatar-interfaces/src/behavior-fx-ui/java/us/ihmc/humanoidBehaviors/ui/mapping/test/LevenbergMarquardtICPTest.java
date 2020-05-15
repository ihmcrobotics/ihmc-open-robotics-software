package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;

import cern.colt.list.BooleanArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.optimization.FunctionOutputCalculator;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;

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

      BooleanArrayList correspondenceFlags = new BooleanArrayList();
      double outlierDistance = 0.2;
      for (int i = 0; i < data1.size(); i++)
      {
         double distance = computeClosestDistance(data1.get(i), fullModel);
         if (distance < outlierDistance)
         {
            correspondenceFlags.add(true);
            drawer.addPoint(data1.get(i), Color.red, true);
         }
         else
         {
            correspondenceFlags.add(false);
            drawer.addPoint(data1.get(i), Color.red, false);
         }
      }

      frame.add(drawer);
      frame.pack();
      frame.setVisible(true);

      ThreadTools.sleepForever();
   }

   @Test
   public void testErrorFunctionDerivationAndJacobian()
   {
      setupPointCloud();

      transformPointCloud(data1, 0.3, 0.5, Math.toRadians(10.0));

      drawer.addPointCloud(fullModel, Color.black, false);
      drawer.addPointCloud(data1, Color.red, false);

      DenseMatrix64F originalError = new DenseMatrix64F(data1.size(), 1);

      BooleanArrayList correspondenceFlags = new BooleanArrayList();
      double outlierDistance = 0.2;
      for (int i = 0; i < data1.size(); i++)
      {
         double distance = computeClosestDistance(data1.get(i), fullModel);
         originalError.set(i, distance);
         if (distance < outlierDistance)
         {
            correspondenceFlags.add(true);
            drawer.addPoint(data1.get(i), Color.red, true);
         }
         else
         {
            correspondenceFlags.add(false);
            drawer.addPoint(data1.get(i), Color.red, false);
         }
      }

      int parameterDimension = 3;
      double perturb = 0.001;
      DenseMatrix64F currentParameter = new DenseMatrix64F(parameterDimension, 1);
      DenseMatrix64F perturbedParameter = new DenseMatrix64F(parameterDimension, 1);
      DenseMatrix64F errorJacobian = new DenseMatrix64F(data1.size(), parameterDimension);

      List<Point2D> purterbedData = new ArrayList<>(data1);
      for (int i = 0; i < parameterDimension; i++)
      {
         for (int j = 0; j < parameterDimension; j++)
         {
            if (j == i)
               perturbedParameter.set(j, 0, currentParameter.get(j, 0) + perturb);
            else
               perturbedParameter.set(j, 0, currentParameter.get(j, 0));
         }
         perturbedParameter.print();
         purterbedData.clear();
         for (int k = 0; k < data1.size(); k++)
            purterbedData.add(new Point2D(data1.get(k)));
         transformPointCloud(purterbedData, perturbedParameter.get(0, 0), perturbedParameter.get(1, 0), perturbedParameter.get(2, 0));
         for (int j = 0; j < data1.size(); j++)
         {
            double distance = computeClosestDistance(purterbedData.get(j), fullModel);
            if (distance < outlierDistance)
            {
               errorJacobian.set(j, i, (distance - originalError.get(j, 0)) / perturb);
            }
            else
            {
               errorJacobian.set(j, i, 0.0);
            }
         }
      }
      errorJacobian.print();
      DenseMatrix64F jacobianTranspose = new DenseMatrix64F(data1.size(), parameterDimension);
      jacobianTranspose.set(errorJacobian);
      CommonOps.transpose(jacobianTranspose);

      DenseMatrix64F squaredJacobian = new DenseMatrix64F(parameterDimension, parameterDimension);
      CommonOps.mult(jacobianTranspose, errorJacobian, squaredJacobian);
      squaredJacobian.print();
      CommonOps.invert(squaredJacobian);
      squaredJacobian.print();

      DenseMatrix64F invMultJacobianTranspose = new DenseMatrix64F(parameterDimension, data1.size());
      CommonOps.mult(squaredJacobian, jacobianTranspose, invMultJacobianTranspose);

      DenseMatrix64F direction = new DenseMatrix64F(parameterDimension, 1);
      CommonOps.mult(invMultJacobianTranspose, originalError, direction);
      direction.print();

      purterbedData.clear();
      for (int k = 0; k < data1.size(); k++)
         purterbedData.add(new Point2D(data1.get(k)));
      transformPointCloud(purterbedData, -direction.get(0, 0), -direction.get(1, 0), -direction.get(2, 0));
      for (int i = 0; i < data1.size(); i++)
      {
         drawer.addPoint(purterbedData.get(i), Color.green, true);
      }

      frame.add(drawer);
      frame.pack();
      frame.setVisible(true);

      ThreadTools.sleepForever();
   }

   @Test
   public void testIteration()
   {
      setupPointCloud();

      transformPointCloud(data1, 0.3, 0.5, Math.toRadians(10.0));

      drawer.addPointCloud(fullModel, Color.black, false);
      drawer.addPointCloud(data1, Color.red, false);

      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(3, data1.size());
      FunctionOutputCalculator functionOutputCalculator = new FunctionOutputCalculator()
      {
         @Override
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
         {
            List<Point2D> transformedData = new ArrayList<>();
            for (int i = 0; i < data1.size(); i++)
               transformedData.add(new Point2D(data1.get(i)));
            transformPointCloud(transformedData, inputParameter.get(0, 0), inputParameter.get(1, 0), inputParameter.get(2, 0));

            DenseMatrix64F errorSpace = new DenseMatrix64F(transformedData.size(), 1);
            for (int i = 0; i < transformedData.size(); i++)
            {
               double distance = computeClosestDistance(transformedData.get(i), fullModel);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }
      };
      DenseMatrix64F purterbationVector = new DenseMatrix64F(3, 1);
      purterbationVector.set(0, 0.001);
      purterbationVector.set(1, 0.001);
      purterbationVector.set(2, 0.001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.setOutputCalculator(functionOutputCalculator);
      boolean isSolved = optimizer.solve(5, 1.0);
      System.out.println("is solved? " + isSolved);

      DenseMatrix64F optimalParameter = optimizer.getOptimalParameter();
      List<Point2D> transformedData = new ArrayList<>();
      for (int i = 0; i < data1.size(); i++)
         transformedData.add(new Point2D(data1.get(i)));
      transformPointCloud(transformedData, optimalParameter.get(0, 0), optimalParameter.get(1, 0), optimalParameter.get(2, 0));

      drawer.addPointCloud(transformedData, Color.GREEN, true);
      
      frame.add(drawer);
      frame.pack();
      frame.setVisible(true);

      ThreadTools.sleepForever();
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
      private final BooleanArrayList fillers;

      private final double xUpper;
      private final double yUpper;
      private final int sizeU;
      private final int sizeV;

      XYPlaneDrawer(double xUpper, double xLower, double yUpper, double yLower)
      {
         pointCloud = new ArrayList<>();
         pointCloudColors = new ArrayList<>();
         fillers = new BooleanArrayList();
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

   @Test
   public void testMatrixOperations()
   {
      DenseMatrix64F testMatrix = new DenseMatrix64F(3, 3);
      testMatrix.set(0, 0, 1);
      testMatrix.set(1, 0, 2);
      testMatrix.set(2, 0, 3);

      testMatrix.set(0, 1, 4);
      testMatrix.set(1, 1, 5);
      testMatrix.set(2, 1, 6);

      testMatrix.set(0, 2, 7);
      testMatrix.set(1, 2, 8);
      testMatrix.set(2, 2, 9);

      System.out.println("testMatrix");
      testMatrix.print();

      DenseMatrix64F matrix2 = new DenseMatrix64F(3, 3);
      matrix2.set(0, 0, 2);
      matrix2.set(1, 0, 0);
      matrix2.set(2, 0, 0);

      matrix2.set(0, 1, 0);
      matrix2.set(1, 1, 2);
      matrix2.set(2, 1, 0);

      matrix2.set(0, 2, 0);
      matrix2.set(1, 2, 0);
      matrix2.set(2, 2, 2);

      System.out.println("matrix2");
      matrix2.print();

      DenseMatrix64F matrix3 = new DenseMatrix64F(3, 3);

      CommonOps.mult(testMatrix, matrix2, matrix3);

      System.out.println("matrix3");
      matrix3.print();
   }
}
