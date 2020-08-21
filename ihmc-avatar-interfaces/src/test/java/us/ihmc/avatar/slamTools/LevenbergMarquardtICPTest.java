package us.ihmc.avatar.slamTools;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.UnaryOperator;

import javax.swing.JFrame;
import javax.swing.JPanel;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import cern.colt.list.BooleanArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.robotics.optimization.OutputCalculator;

/**
 * see {@link LevenbergMarquardtParameterOptimizer}.
 */
@Tag("point-cloud-drift-correction-test")
public class LevenbergMarquardtICPTest
{
   private boolean visualize = false;
   private XYPlaneDrawer drawer;
   private JFrame frame;

   private List<Point2D> fullModel = new ArrayList<>();
   private List<Point2D> data1 = new ArrayList<>();
   private List<Point2D> data2 = new ArrayList<>();

   private Function<DMatrixRMaj, RigidBodyTransform> inputFunction;

   private double innerCircleLong = 2.0;
   private double innerCircleShort = 1.0;
   private double outterCircleLong = 4.0;
   private double outterCircleShort = 3.0;

   private void setupPointCloud()
   {
      drawer = new XYPlaneDrawer(10.0, -10.0, 10.0, -10.0);
      frame = new JFrame("2D_LM_ICP_TEST");

      frame.setPreferredSize(drawer.getDimension());
      frame.setLocation(200, 100);

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

      inputFunction = new Function<DMatrixRMaj, RigidBodyTransform>()
      {
         @Override
         public RigidBodyTransform apply(DMatrixRMaj input)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            transform.setRotationYawAndZeroTranslation(input.get(2));
            transform.getTranslation().set(input.get(0), input.get(1), 0.0);
            return transform;
         }
      };
      assertTrue(true);
   }

   @Test
   public void testVisualization()
   {
      setupPointCloud();

      DMatrixRMaj drift1 = new DMatrixRMaj(3, 1);
      drift1.set(0, 1.0);
      drift1.set(1, 2.0);
      drift1.set(2, Math.toRadians(30.0));
      transformPointCloud(data1, inputFunction.apply(drift1));

      DMatrixRMaj drift2 = new DMatrixRMaj(3, 1);
      drift2.set(0, -1.0);
      drift2.set(1, -2.0);
      drift2.set(2, Math.toRadians(-90.0));
      transformPointCloud(data2, inputFunction.apply(drift2));

      drawer.addPointCloud(fullModel, Color.black, false);
      drawer.addPointCloud(data1, Color.red, false);
      drawer.addPointCloud(data2, Color.green, false);

      if (visualize)
      {
         frame.add(drawer);
         frame.pack();
         frame.setVisible(true);
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testFindingClosestPointWithFullModel()
   {
      setupPointCloud();

      double driftX = 0.3;
      double driftY = 0.5;
      double driftTheta = Math.toRadians(10.0);
      DMatrixRMaj driftSpace = new DMatrixRMaj(3, 1);
      driftSpace.set(0, driftX);
      driftSpace.set(1, driftY);
      driftSpace.set(2, driftTheta);
      transformPointCloud(data1, inputFunction.apply(driftSpace));

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

      if (visualize)
      {
         frame.add(drawer);
         frame.pack();
         frame.setVisible(true);
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testErrorFunctionDerivationAndJacobian()
   {
      setupPointCloud();

      double driftX = 0.3;
      double driftY = 0.5;
      double driftTheta = Math.toRadians(10.0);
      DMatrixRMaj driftSpace = new DMatrixRMaj(3, 1);
      driftSpace.set(0, driftX);
      driftSpace.set(1, driftY);
      driftSpace.set(2, driftTheta);
      transformPointCloud(data1, inputFunction.apply(driftSpace));

      drawer.addPointCloud(fullModel, Color.black, false);
      drawer.addPointCloud(data1, Color.red, false);

      DMatrixRMaj originalError = new DMatrixRMaj(data1.size(), 1);

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
      DMatrixRMaj currentParameter = new DMatrixRMaj(parameterDimension, 1);
      DMatrixRMaj perturbedParameter = new DMatrixRMaj(parameterDimension, 1);
      DMatrixRMaj errorJacobian = new DMatrixRMaj(data1.size(), parameterDimension);

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
         purterbedData.clear();
         for (int k = 0; k < data1.size(); k++)
            purterbedData.add(new Point2D(data1.get(k)));
         transformPointCloud(purterbedData, inputFunction.apply(perturbedParameter));
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
      DMatrixRMaj jacobianTranspose = new DMatrixRMaj(data1.size(), parameterDimension);
      jacobianTranspose.set(errorJacobian);
      CommonOps_DDRM.transpose(jacobianTranspose);

      DMatrixRMaj squaredJacobian = new DMatrixRMaj(parameterDimension, parameterDimension);
      CommonOps_DDRM.mult(jacobianTranspose, errorJacobian, squaredJacobian);
      CommonOps_DDRM.invert(squaredJacobian);

      DMatrixRMaj invMultJacobianTranspose = new DMatrixRMaj(parameterDimension, data1.size());
      CommonOps_DDRM.mult(squaredJacobian, jacobianTranspose, invMultJacobianTranspose);

      DMatrixRMaj direction = new DMatrixRMaj(parameterDimension, 1);
      CommonOps_DDRM.mult(invMultJacobianTranspose, originalError, direction);
      System.out.println("direction of the optimization is,");
      direction.print();

      assertTrue(direction.get(0) > 0.0, "direction of the translation x     is correct.");
      assertTrue(direction.get(1) > 0.0, "direction of the translation y     is correct.");
      assertTrue(direction.get(2) > 0.0, "direction of the translation theta is correct.");

      // negate
      for (int i = 0; i < direction.data.length; i++)
         direction.set(i, -direction.data[i]);

      purterbedData.clear();
      for (int k = 0; k < data1.size(); k++)
         purterbedData.add(new Point2D(data1.get(k)));
      transformPointCloud(purterbedData, inputFunction.apply(direction));
      for (int i = 0; i < data1.size(); i++)
      {
         drawer.addPoint(purterbedData.get(i), Color.green, true);
      }

      if (visualize)
      {
         frame.add(drawer);
         frame.pack();
         frame.setVisible(true);
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testIteration()
   {
      setupPointCloud();

      double driftX = -0.3;
      double driftY = 0.5;
      double driftTheta = Math.toRadians(-30.0);
      DMatrixRMaj driftSpace = new DMatrixRMaj(3, 1);
      driftSpace.set(0, driftX);
      driftSpace.set(1, driftY);
      driftSpace.set(2, driftTheta);

      transformPointCloud(fullModel, inputFunction.apply(driftSpace));

      drawer.addPointCloud(fullModel, Color.black, false);
      drawer.addPointCloud(data1, Color.red, false);

      OutputCalculator outputCalculator = new OutputCalculator()
      {
         @Override
         public DMatrixRMaj apply(DMatrixRMaj inputParameter)
         {
            List<Point2D> transformedData = new ArrayList<>();
            for (int i = 0; i < data1.size(); i++)
               transformedData.add(new Point2D(data1.get(i)));
            transformPointCloud(transformedData, inputFunction.apply(inputParameter));

            DMatrixRMaj errorSpace = new DMatrixRMaj(transformedData.size(), 1);
            for (int i = 0; i < transformedData.size(); i++)
            {
               double distance = computeClosestDistance(transformedData.get(i), fullModel);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }
      };
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(inputFunction, outputCalculator, 3, data1.size());
      DMatrixRMaj purterbationVector = new DMatrixRMaj(3, 1);
      purterbationVector.set(0, 0.00001);
      purterbationVector.set(1, 0.00001);
      purterbationVector.set(2, 0.00001);
      optimizer.setPerturbationVector(purterbationVector);
      boolean isSolved = false;
      for (int i = 0; i < 30; i++)
      {
         optimizer.iterate();
         if (optimizer.getQuality() < 0.4)
         {
            isSolved = true;
            break;
         }
         System.out.println(i + " " + optimizer.getQuality());
      }
      System.out.println("is solved? " + isSolved + " " + optimizer.getIteration() + " " + optimizer.getQuality());
      optimizer.getOptimalParameter().print();

      DMatrixRMaj optimalParameter = optimizer.getOptimalParameter();
      List<Point2D> transformedData = new ArrayList<>();
      for (int i = 0; i < data1.size(); i++)
         transformedData.add(new Point2D(data1.get(i)));
      transformPointCloud(transformedData, inputFunction.apply(optimalParameter));

      Point2D aPointOfDoughnut = new Point2D(0.0, innerCircleLong);
      Point2D driftedPoint = new Point2D(aPointOfDoughnut);
      Point2D correctedPoint = new Point2D(aPointOfDoughnut);

      transformPoint(driftedPoint, inputFunction.apply(driftSpace));
      transformPoint(correctedPoint, inputFunction.apply(optimalParameter));
      assertTrue(correctedPoint.distance(driftedPoint) < 0.06, "a point on the drifted doughnut corrected with icp. " + correctedPoint.distance(driftedPoint));

      drawer.addPointCloud(transformedData, Color.green, true);

      if (visualize)
      {
         frame.add(drawer);
         frame.pack();
         frame.setVisible(true);
         ThreadTools.sleepForever();
      }
   }

   /**
    * method finding the closest point for the given point with brute force.
    * for point cloud data, we will use k-d tree (joctomap).
    */
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

   private void transformPointCloud(List<Point2D> pointCloud, RigidBodyTransform transformer)
   {
      pointCloud.forEach(point -> transformPoint(point, transformer));
   }

   private void transformPoint(Point2D point, RigidBodyTransform transformer)
   {
      transformer.transform(point);
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
}
