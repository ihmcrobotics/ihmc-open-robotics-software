package us.ihmc.avatar.slamTools;

import cern.colt.list.BooleanArrayList;
import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import rosgraph_msgs.Log;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.robotics.optimization.OutputCalculator;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.*;

/**
 * see {@link LevenbergMarquardtParameterOptimizer}.
 */
public class LevenbergMarquardtICPTest
{
   private boolean visualize = false;
   private boolean runDrawer = false;

   private XYPlaneDrawer drawer;
   private JFrame frame;

   private List<Point2D> fullModel = new ArrayList<>();
   private List<Point2D> modelSample1 = new ArrayList<>();
   private List<Point2D> modelSample2 = new ArrayList<>();

   private Function<DMatrixRMaj, RigidBodyTransform> transformFromMatrixFunction;

   private double innerCircleLong = 2.0;
   private double innerCircleShort = 1.0;
   private double outterCircleLong = 4.0;
   private double outterCircleShort = 3.0;

   @BeforeEach
   public void setup()
   {
      visualize &= !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
      runDrawer &= !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   private void setupPointCloud()
   {
      if (runDrawer)
      {
         drawer = new XYPlaneDrawer(10.0, -10.0, 10.0, -10.0);
         frame = new JFrame("2D_LM_ICP_TEST");

         frame.setPreferredSize(drawer.getDimension());
         frame.setLocation(200, 100);
      }

      fullModel.addAll(generatePointsOnEllipsoid(50, Math.toRadians(90.0), Math.toRadians(359.9), innerCircleLong, innerCircleShort));
      fullModel.addAll(generatePointsOnEllipsoid(70, Math.toRadians(90.0), Math.toRadians(359.9), outterCircleLong, outterCircleShort));
      fullModel.addAll(generatePointsOnLine(10, innerCircleShort, outterCircleShort, 0.0, true));
      fullModel.addAll(generatePointsOnLine(10, innerCircleLong, outterCircleLong, 0.0, false));

      modelSample1.addAll(generatePointsOnEllipsoid(35, Math.toRadians(90.0), Math.toRadians(200.0), innerCircleLong, innerCircleShort));
      modelSample1.addAll(generatePointsOnEllipsoid(50, Math.toRadians(90.0), Math.toRadians(320.0), outterCircleLong, outterCircleShort));
      modelSample1.addAll(generatePointsOnLine(10, innerCircleShort, outterCircleShort, 0.0, true));

      modelSample2.addAll(generatePointsOnEllipsoid(50, Math.toRadians(130.0), Math.toRadians(359.9), innerCircleLong, innerCircleShort));
      modelSample2.addAll(generatePointsOnEllipsoid(60, Math.toRadians(120.0), Math.toRadians(359.9), outterCircleLong, outterCircleShort));
      modelSample2.addAll(generatePointsOnLine(10, innerCircleLong, outterCircleLong, 0.0, false));

      transformFromMatrixFunction = new Function<DMatrixRMaj, RigidBodyTransform>()
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
   public void testForwardBackwardsInputConverter()
   {
      Function<DMatrixRMaj, RigidBodyTransform> spatialInputFunction = LevenbergMarquardtParameterOptimizer.createSpatialInputFunction(false);
      Function<RigidBodyTransformReadOnly, DMatrixRMaj> inverseSpatialInputFunction = LevenbergMarquardtParameterOptimizer.createInverseSpatialInputFunction(
            false);

      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         RigidBodyTransform transformOriginal = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         DMatrixRMaj input = inverseSpatialInputFunction.apply(transformOriginal);
         RigidBodyTransform reconstructedTransform = spatialInputFunction.apply(input);

         EuclidCoreTestTools.assertVector3DGeometricallyEquals(transformOriginal.getTranslation(), reconstructedTransform.getTranslation(), 1e-7);
         assertTrue(
               AngleTools.computeAngleDifferenceMinusPiToPi(transformOriginal.getRotation().getYaw(), reconstructedTransform.getRotation().getYaw()) < 1e-7);
         //         EuclidCoreTestTools.assertEquals(transformOriginal, reconstructedTransform, 1e-7);

         DMatrixRMaj randomInput = new DMatrixRMaj(4, 1);
         randomInput.set(0, RandomNumbers.nextDouble(random, 100));
         randomInput.set(1, RandomNumbers.nextDouble(random, 100));
         randomInput.set(2, RandomNumbers.nextDouble(random, 100));
         randomInput.set(3, RandomNumbers.nextDouble(random, Math.PI));

         transformOriginal = spatialInputFunction.apply(randomInput);
         input = inverseSpatialInputFunction.apply(transformOriginal);

         EjmlUnitTests.assertEquals(randomInput, input, 1e-7);
      }

      spatialInputFunction = LevenbergMarquardtParameterOptimizer.createSpatialInputFunction(true);
      inverseSpatialInputFunction = LevenbergMarquardtParameterOptimizer.createInverseSpatialInputFunction(true);

      for (int i = 0; i < 1000; i++)
      {
         RigidBodyTransform transformOriginal = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         DMatrixRMaj input = inverseSpatialInputFunction.apply(transformOriginal);
         RigidBodyTransform reconstructedTransform = spatialInputFunction.apply(input);

         EuclidCoreTestTools.assertEquals(transformOriginal, reconstructedTransform, 1e-7);

         DMatrixRMaj randomInput = new DMatrixRMaj(6, 1);
         randomInput.set(0, RandomNumbers.nextDouble(random, 100));
         randomInput.set(1, RandomNumbers.nextDouble(random, 100));
         randomInput.set(2, RandomNumbers.nextDouble(random, 100));
         randomInput.set(3, RandomNumbers.nextDouble(random, Math.PI));
         randomInput.set(4, RandomNumbers.nextDouble(random, Math.PI));
         randomInput.set(5, RandomNumbers.nextDouble(random, Math.PI));

         transformOriginal = spatialInputFunction.apply(randomInput);
         input = inverseSpatialInputFunction.apply(transformOriginal);

         Quaternion orientationA = new Quaternion(randomInput.get(3), randomInput.get(4), randomInput.get(5));
         Quaternion orientationB = new Quaternion(input.get(3), input.get(4), input.get(5));

         EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(orientationA, orientationB, 1e-7);
      }
   }

   @Test
   public void testVisualization()
   {
      setupPointCloud();

      DMatrixRMaj drift1 = new DMatrixRMaj(3, 1);
      drift1.set(0, 1.0);
      drift1.set(1, 2.0);
      drift1.set(2, Math.toRadians(30.0));
      transformPointCloud(modelSample1, transformFromMatrixFunction.apply(drift1));

      DMatrixRMaj drift2 = new DMatrixRMaj(3, 1);
      drift2.set(0, -1.0);
      drift2.set(1, -2.0);
      drift2.set(2, Math.toRadians(-90.0));
      transformPointCloud(modelSample2, transformFromMatrixFunction.apply(drift2));

      if (runDrawer)
      {
         drawer.addPointCloud(fullModel, Color.black, false);
         drawer.addPointCloud(modelSample1, Color.red, false);
         drawer.addPointCloud(modelSample2, Color.green, false);
      }

      if (visualize && runDrawer)
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
      transformPointCloud(modelSample1, transformFromMatrixFunction.apply(driftSpace));

      if (runDrawer)
      {
         drawer.addPointCloud(fullModel, Color.black, false);
         drawer.addPointCloud(modelSample1, Color.red, false);
      }

      BooleanArrayList correspondenceFlags = new BooleanArrayList();
      double outlierDistance = 0.2;
      for (int i = 0; i < modelSample1.size(); i++)
      {
         double distance = computeClosestDistance(modelSample1.get(i), fullModel);
         if (distance < outlierDistance)
         {
            correspondenceFlags.add(true);
            if (runDrawer)
               drawer.addPoint(modelSample1.get(i), Color.red, true);
         }
         else
         {
            correspondenceFlags.add(false);
            if (runDrawer)
               drawer.addPoint(modelSample1.get(i), Color.red, false);
         }
      }

      if (visualize && runDrawer)
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
      transformPointCloud(modelSample1, transformFromMatrixFunction.apply(driftSpace));

      if (runDrawer)
      {
         drawer.addPointCloud(fullModel, Color.black, false);
         drawer.addPointCloud(modelSample1, Color.red, false);
      }

      DMatrixRMaj originalError = new DMatrixRMaj(modelSample1.size(), 1);

      BooleanArrayList correspondenceFlags = new BooleanArrayList();
      double outlierDistance = 0.2;
      for (int i = 0; i < modelSample1.size(); i++)
      {
         double distance = computeClosestDistance(modelSample1.get(i), fullModel);
         originalError.set(i, distance);
         if (distance < outlierDistance)
         {
            correspondenceFlags.add(true);
            if (runDrawer)
               drawer.addPoint(modelSample1.get(i), Color.red, true);
         }
         else
         {
            correspondenceFlags.add(false);
            if (runDrawer)
               drawer.addPoint(modelSample1.get(i), Color.red, false);
         }
      }

      int parameterDimension = 3;
      double perturb = 0.001;
      DMatrixRMaj currentParameter = new DMatrixRMaj(parameterDimension, 1);
      DMatrixRMaj perturbedParameter = new DMatrixRMaj(parameterDimension, 1);
      DMatrixRMaj errorJacobian = new DMatrixRMaj(modelSample1.size(), parameterDimension);

      List<Point2D> purterbedData = new ArrayList<>(modelSample1);
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
         for (int k = 0; k < modelSample1.size(); k++)
            purterbedData.add(new Point2D(modelSample1.get(k)));
         transformPointCloud(purterbedData, transformFromMatrixFunction.apply(perturbedParameter));
         for (int j = 0; j < modelSample1.size(); j++)
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
      DMatrixRMaj jacobianTranspose = new DMatrixRMaj(modelSample1.size(), parameterDimension);
      jacobianTranspose.set(errorJacobian);
      CommonOps_DDRM.transpose(jacobianTranspose);

      DMatrixRMaj squaredJacobian = new DMatrixRMaj(parameterDimension, parameterDimension);
      CommonOps_DDRM.mult(jacobianTranspose, errorJacobian, squaredJacobian);
      CommonOps_DDRM.invert(squaredJacobian);

      DMatrixRMaj invMultJacobianTranspose = new DMatrixRMaj(parameterDimension, modelSample1.size());
      CommonOps_DDRM.mult(squaredJacobian, jacobianTranspose, invMultJacobianTranspose);

      DMatrixRMaj direction = new DMatrixRMaj(parameterDimension, 1);
      CommonOps_DDRM.mult(invMultJacobianTranspose, originalError, direction);
      System.out.println("direction of the optimization is,");
      direction.print();

      assertTrue(direction.get(0) > 0.0, "direction of the translation x     is correct.");
      assertTrue(direction.get(1) > 0.0, "direction of the translation y     is correct.");
      assertTrue(direction.get(2) > 0.0, "direction of the translation theta is correct.");

      if (runDrawer)
      {
         // negate
         for (int i = 0; i < direction.data.length; i++)
            direction.set(i, -direction.data[i]);

         purterbedData.clear();
         for (int k = 0; k < modelSample1.size(); k++)
            purterbedData.add(new Point2D(modelSample1.get(k)));
         transformPointCloud(purterbedData, transformFromMatrixFunction.apply(direction));
         for (int i = 0; i < modelSample1.size(); i++)
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
   }

   @Test
   public void testIteration()
   {
      setupPointCloud();

      // Transform the baseline data. This makes the base data move away from the zero position, and the ICP algorithm should then be able to estimate that this
      // is the transformation that it has to apply to correct the sensor data.
      double driftX = -0.3;
      double driftY = 0.5;
      double driftTheta = Math.toRadians(-30.0);
      DMatrixRMaj driftToApply = new DMatrixRMaj(3, 1);
      driftToApply.set(0, driftX);
      driftToApply.set(1, driftY);
      driftToApply.set(2, driftTheta);
      RigidBodyTransformReadOnly driftTransform = transformFromMatrixFunction.apply(driftToApply);

      // Force the data from the full model to drift.
      transformPointCloud(fullModel, driftTransform);

      if (runDrawer)
      {
         drawer.addPointCloud(fullModel, Color.black, false);
         drawer.addPointCloud(modelSample1, Color.red, false);
      }

      // Set up the method that computes the error space based on the input data.
      OutputCalculator outputCalculator = new OutputCalculator()
      {
         @Override
         public DMatrixRMaj apply(DMatrixRMaj inputParameter)
         {
            // apply the transform to a copy of the sampled data
            List<Point2D> transformedData = modelSample1.stream().map(Point2D::new).toList();
            RigidBodyTransformReadOnly inputParameterTransform = transformFromMatrixFunction.apply(inputParameter);
            transformPointCloud(transformedData, inputParameterTransform);

            // compute the minimal distance from this point to the "model" data, which is the data that's been drifed.
            DMatrixRMaj errorSpace = new DMatrixRMaj(transformedData.size(), 1);
            for (int i = 0; i < transformedData.size(); i++)
            {
               double distance = computeClosestDistance(transformedData.get(i), fullModel);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }
      };

      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(transformFromMatrixFunction, outputCalculator, 3, modelSample1.size());
      DMatrixRMaj purterbationVector = new DMatrixRMaj(3, 1);
      purterbationVector.set(0, 0.00001);
      purterbationVector.set(1, 0.00001);
      purterbationVector.set(2, 0.00001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.initialize();
      boolean converged = false;
      for (int i = 0; i < 30; i++)
      {
         optimizer.iterate();
         LogTools.info("Optimizer iteration {} resulted in {} quality", i, optimizer.getQuality());

         if (optimizer.getQuality() < 0.4)
         {
            LogTools.info("Found convergence on iteration {}", i);
            converged = true;
            break;
         }
      }
      LogTools.info("Found solution? {}. Converged in {} iterations with {} quality", converged, optimizer.getIteration(), optimizer.getQuality());
      RigidBodyTransformReadOnly optimalTransform = transformFromMatrixFunction.apply(optimizer.getOptimalParameter());
      LogTools.info("The computed transform is \n" + optimalTransform);

      // Create a deep copy of the data 1 array.
      List<Point2D> correctedData = modelSample1.stream().map(Point2D::new).toList();

      // get the transform that was calculated by the optimizer.
      transformPointCloud(correctedData, optimalTransform);

      Point2D aPointOfDoughnut = new Point2D(0.0, innerCircleLong);
      Point2D driftedPoint = new Point2D(aPointOfDoughnut);
      Point2D correctedPoint = new Point2D(aPointOfDoughnut);

      transformPoint(driftedPoint, driftTransform);
      transformPoint(correctedPoint, optimalTransform);

      if (runDrawer)
      {
         drawer.addPointCloud(correctedData, Color.green, true);

         if (visualize)
         {
            frame.add(drawer);
            frame.pack();
            frame.setVisible(true);
            ThreadTools.sleepForever();
         }
      }

      assertTrue(correctedPoint.distance(driftedPoint) < 0.06, "a point on the drifted doughnut corrected with icp. " + correctedPoint.distance(driftedPoint));

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

   private void transformPointCloud(List<Point2D> pointCloud, RigidBodyTransformReadOnly transformer)
   {
      pointCloud.forEach(point -> transformPoint(point, transformer));
   }

   private void transformPoint(Point2D point, RigidBodyTransformReadOnly transformer)
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
