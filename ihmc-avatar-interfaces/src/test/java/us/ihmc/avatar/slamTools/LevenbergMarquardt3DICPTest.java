package us.ihmc.avatar.slamTools;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.SLAMViewer;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.slam.tools.PLYasciiFormatFormatDataImporter;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.optimization.FunctionOutputCalculator;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;

public class LevenbergMarquardt3DICPTest
{
   private boolean visualize = true;

   @Test
   public void visualizeDriftedCowModel()
   {
      String cowPLYPath = "C:\\PointCloudData\\PLY\\Cow\\cow.ply";
      File cowPointCloudFile = new File(cowPLYPath);
      Point3D[] originalCowPointCloud = PLYasciiFormatFormatDataImporter.getPointsFromFile(cowPointCloudFile);
      Point3D[] driftedCowPointCloud = new Point3D[originalCowPointCloud.length];

      RigidBodyTransform driftingTransform = new RigidBodyTransform();
      driftingTransform.setTranslationAndIdentityRotation(0.0, 0.0, 0.1);
      driftingTransform.appendRollRotation(Math.toRadians(10.0));
      for (int i = 0; i < driftedCowPointCloud.length; i++)
      {
         driftedCowPointCloud[i] = new Point3D(originalCowPointCloud[i]);
         driftingTransform.transform(driftedCowPointCloud[i]);
      }

      if (visualize)
      {
         SLAMViewer slamViewer = new SLAMViewer();

         slamViewer.addPointCloud(originalCowPointCloud, Color.GREEN, 0.075);
         slamViewer.addPointCloud(driftedCowPointCloud, Color.BLUE, 0.075);

         slamViewer.start("visualizeDriftedCowModel");
         ThreadTools.sleepForever();
      }

      assertTrue(true);
   }

   @Test
   public void testCorrespondence()
   {
      String cowPLYPath = "C:\\PointCloudData\\PLY\\Cow\\cow.ply";
      File cowPointCloudFile = new File(cowPLYPath);
      Point3D[] originalCowPointCloud = PLYasciiFormatFormatDataImporter.getPointsFromFile(cowPointCloudFile);
      Point3D[] driftedCowPointCloud = new Point3D[originalCowPointCloud.length];

      RigidBodyTransform driftingTransform = new RigidBodyTransform();
      driftingTransform.setTranslationAndIdentityRotation(0.0, 0.0, 0.1);
      driftingTransform.appendRollRotation(Math.toRadians(10.0));
      for (int i = 0; i < driftedCowPointCloud.length; i++)
      {
         driftedCowPointCloud[i] = new Point3D(originalCowPointCloud[i]);
         driftingTransform.transform(driftedCowPointCloud[i]);
      }

      // octree.
      Point3D dummySensorLocation = new Point3D(0.0, 0.0, 2.0);
      Point3DReadOnly[] pointCloud = originalCowPointCloud;

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = pointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(pointCloud, dummySensorLocation));

      double octreeResolution = 0.02;
      NormalOcTree octree = new NormalOcTree(octreeResolution);
      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);

      // pairing corresponding points.
      Map<Point3D, Point3D> correspondingPairs = new HashMap<>();
      for (int i = 0; i < driftedCowPointCloud.length; i++)
      {
         Point3D pairedPoint = new Point3D();
         double distance = SLAMTools.computeDistanceToNormalOctreeAndPackCorrespondingPoint(octree, driftedCowPointCloud[i], 10, pairedPoint);

         correspondingPairs.put(driftedCowPointCloud[i], pairedPoint);
      }

      if (visualize)
      {
         SLAMViewer slamViewer = new SLAMViewer();

         slamViewer.addPointCloud(driftedCowPointCloud, Color.BLUE, 0.01);
         slamViewer.addOctree(octree, Color.RED, octreeResolution, true);

         //         for (int i = 0; i < driftedCowPointCloud.length; i++)
         //            slamViewer.addLines(driftedCowPointCloud[i], correspondingPairs.get(driftedCowPointCloud[i]), Color.LIMEGREEN, 0.005);

         slamViewer.start("visualizeDriftedCowModel");
         ThreadTools.sleepForever();
      }

      assertTrue(true);
   }

   @Test
   public void testICP()
   {
      String cowPLYPath = "C:\\PointCloudData\\PLY\\Cow\\cow.ply";
      File cowPointCloudFile = new File(cowPLYPath);
      Point3D[] originalCowPointCloud = PLYasciiFormatFormatDataImporter.getPointsFromFile(cowPointCloudFile);
      Point3D[] driftedCowPointCloud = new Point3D[originalCowPointCloud.length];

      RigidBodyTransform driftingTransform = new RigidBodyTransform();
      driftingTransform.setTranslationAndIdentityRotation(0.0, 0.0, 0.1);
      driftingTransform.appendRollRotation(Math.toRadians(30.0));
      driftingTransform.appendPitchRotation(Math.toRadians(-10.0));
      driftingTransform.appendRollRotation(Math.toRadians(5.0));
      for (int i = 0; i < driftedCowPointCloud.length; i++)
      {
         driftedCowPointCloud[i] = new Point3D(originalCowPointCloud[i]);
         driftingTransform.transform(driftedCowPointCloud[i]);
      }

      // octree.
      Point3D dummySensorLocation = new Point3D(0.0, 0.0, 2.0);
      Point3DReadOnly[] pointCloud = originalCowPointCloud;

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = pointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(pointCloud, dummySensorLocation));

      double octreeResolution = 0.02;
      NormalOcTree octree = new NormalOcTree(octreeResolution);
      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);

      // define optimizer.
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(6, driftedCowPointCloud.length);
      FunctionOutputCalculator functionOutputCalculator = new FunctionOutputCalculator()
      {
         @Override
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
         {
            Point3D[] transformedData = new Point3D[driftedCowPointCloud.length];
            for (int i = 0; i < driftedCowPointCloud.length; i++)
               transformedData[i] = new Point3D(driftedCowPointCloud[i]);
            transformPointCloud(transformedData, inputParameter.getData());

            DenseMatrix64F errorSpace = new DenseMatrix64F(transformedData.length, 1);
            for (int i = 0; i < transformedData.length; i++)
            {
               double distance = computeClosestDistance(transformedData[i], originalCowPointCloud);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Point3D point, Point3D[] originalCowPointCloud)
         {
            double distance = SLAMTools.computeDistanceToNormalOctree(octree, point, 10);
            return distance;
         }
      };

      DenseMatrix64F purterbationVector = new DenseMatrix64F(6, 1);
      purterbationVector.set(0, 0.00001);
      purterbationVector.set(1, 0.00001);
      purterbationVector.set(2, 0.00001);
      purterbationVector.set(3, 0.00001);
      purterbationVector.set(4, 0.00001);
      purterbationVector.set(5, 0.00001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.setOutputCalculator(functionOutputCalculator);
      boolean isSolved = optimizer.solve(25, 0.1);
      LogTools.info("Computation is done " + optimizer.getComputationTime() + " sec.");
      System.out.println("is solved? " + isSolved + " " + optimizer.getQuality());
      optimizer.getOptimalParameter().print();

      DenseMatrix64F optimalParameter = optimizer.getOptimalParameter();
      Point3D[] transformedData = new Point3D[driftedCowPointCloud.length];
      for (int i = 0; i < driftedCowPointCloud.length; i++)
         transformedData[i] = new Point3D(driftedCowPointCloud[i]);
      transformPointCloud(transformedData, optimalParameter.getData());

      if (visualize)
      {
         SLAMViewer slamViewer = new SLAMViewer();

         slamViewer.addPointCloud(driftedCowPointCloud, Color.BLUE, 0.01);
         slamViewer.addOctree(octree, Color.RED, octreeResolution, true);
         slamViewer.addPointCloud(transformedData, Color.GREEN, 0.02);

         slamViewer.start("visualizeDriftedCowModel");
         ThreadTools.sleepForever();
      }

      assertTrue(true);
   }

   private void transformPointCloud(Point3D[] pointCloud, double... transformParameters)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationAndIdentityRotation(transformParameters[0], transformParameters[1], transformParameters[2]);
      transform.appendRollRotation(transformParameters[3]);
      transform.appendPitchRotation(transformParameters[4]);
      transform.appendYawRotation(transformParameters[5]);
      for (int i = 0; i < pointCloud.length; i++)
      {
         Point3D point = pointCloud[i];
         transform.transform(point);
      }
   }
}
