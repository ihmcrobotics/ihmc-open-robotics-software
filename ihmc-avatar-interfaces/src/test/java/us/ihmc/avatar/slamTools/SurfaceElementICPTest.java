package us.ihmc.avatar.slamTools;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.util.List;
import java.util.function.Function;
import java.util.function.UnaryOperator;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.SLAMViewer;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataLoader;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;
import us.ihmc.robotics.optimization.OutputCalculator;

@Tag("point-cloud-drift-correction-test")
public class SurfaceElementICPTest
{
   private static final boolean VISUALIZE = true;

   @Test
   public void testSurfaceElements()
   {
      DriftCase driftCase = DriftCase.YDrift;
      String driftCasePath = driftCase.getFilePath();
      File pointCloudFile = new File(driftCasePath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(octreeResolution);
      slam.addKeyFrame(messages.get(0), true);

      NormalOcTree map = slam.getMapOcTree();
      map.updateNormals();

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(10);
      SLAMFrame frame2 = new SLAMFrame(slam.getLatestFrame(), messages.get(1), normalEstimationParameters);
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.05;
      int minimumNumberOfHits = 10;
      boolean updateNormal = false;
      int maxNumberOfSurfels = Integer.MAX_VALUE;
      frame2.registerSurfaceElements(map, windowMargin, surfaceElementResolution, minimumNumberOfHits, updateNormal, maxNumberOfSurfels);

      if (VISUALIZE)
      {
         SLAMViewer slamViewer = new SLAMViewer();
         slamViewer.addOctree(map, Color.CORAL, octreeResolution, true);
         slamViewer.addOctree(map, Color.CORAL, octreeResolution);
         slamViewer.addOctree(frame2.getFrameMap(), Color.GREEN, surfaceElementResolution, !updateNormal);

         slamViewer.start("testSurfaceElements");
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testDistanceComputation()
   {
      DriftCase driftCase = DriftCase.YDrift;
      String driftCasePath = driftCase.getFilePath();
      File pointCloudFile = new File(driftCasePath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(octreeResolution);
      slam.addKeyFrame(messages.get(0), true);

      NormalOcTree map = slam.getMapOcTree();
      map.updateNormals();

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(10);
      SLAMFrame frame2 = new SLAMFrame(slam.getLatestFrame(), messages.get(1), normalEstimationParameters);
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.05;
      int minimumNumberOfHits = 1;
      boolean updateNormal = false;
      int maxNumberOfSurfels = Integer.MAX_VALUE;
      frame2.registerSurfaceElements(map, windowMargin, surfaceElementResolution, minimumNumberOfHits, updateNormal, maxNumberOfSurfels);

      int numberOfBigPointsToVisualize = 20;
      TDoubleArrayList bigSurfelDistances = new TDoubleArrayList(numberOfBigPointsToVisualize);
      TIntArrayList bigSurfelIndices = new TIntArrayList(numberOfBigPointsToVisualize);
      for (int i = 0; i < numberOfBigPointsToVisualize; i++)
      {
         bigSurfelDistances.add(0.0);
         bigSurfelIndices.add(0);
      }

      int numberOfSurfel = frame2.getSurfaceElements().size();
      LogTools.info("numberOfSurfel " + numberOfSurfel);
      for (int i = 0; i < numberOfSurfel; i++)
      {
         double distance = SLAMTools.computeDistancePointToNormalOctree(map, frame2.getSurfaceElements().get(i).getPoint());
         System.out.println(distance);
         for (int j = 0; j < bigSurfelDistances.size(); j++)
         {
            double bigDistance = bigSurfelDistances.get(j);
            if (distance > bigDistance)
            {
               for (int k = bigSurfelDistances.size() - 1; k > j; k--)
               {
                  bigSurfelDistances.set(k, bigSurfelDistances.get(k - 1));
                  bigSurfelIndices.set(k, bigSurfelIndices.get(k - 1));
               }
               bigSurfelDistances.set(j, distance);
               bigSurfelIndices.set(j, i);
               break;
            }
         }
      }
      Point3DReadOnly[] bigPoints = new Point3D[numberOfBigPointsToVisualize];
      for (int i = 0; i < bigSurfelDistances.size(); i++)
      {
         bigPoints[i] = new Point3D(frame2.getSurfaceElements().get(bigSurfelIndices.get(i)).getPoint());
         System.out.println(i + " " + bigSurfelIndices.get(i) + " " + bigSurfelDistances.get(i) + " " + bigPoints[i]);
      }

      if (VISUALIZE)
      {
         SLAMViewer slamViewer = new SLAMViewer();
         slamViewer.addOctree(map, Color.CORAL, octreeResolution, true);
         slamViewer.addOctree(map, Color.CORAL, octreeResolution);
         slamViewer.addOctree(frame2.getFrameMap(), Color.GREEN, surfaceElementResolution, !updateNormal);
         slamViewer.addPointCloud(bigPoints, Color.RED, 0.02);

         slamViewer.start("testDistanceComputation");
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testDriftCorrection()
   {
      DriftCase driftCase = DriftCase.YDrift;
      String driftCasePath = driftCase.getFilePath();
      File pointCloudFile = new File(driftCasePath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(octreeResolution);
      slam.addKeyFrame(messages.get(0), true);

      NormalOcTree map = slam.getMapOcTree();
      map.updateNormals();

      NormalEstimationParameters frameNormalEstimationParameters = new NormalEstimationParameters();
      frameNormalEstimationParameters.setNumberOfIterations(10);

      SLAMFrame frame2 = new SLAMFrame(slam.getLatestFrame(), messages.get(1), frameNormalEstimationParameters);
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.04;
      int minimumNumberOfHits = 3;
      boolean updateNormal = false;
      int maxNumberOfSurfels = Integer.MAX_VALUE;
      frame2.registerSurfaceElements(map, windowMargin, surfaceElementResolution, minimumNumberOfHits, updateNormal, maxNumberOfSurfels);

      if (VISUALIZE)
      {
         SLAMViewer originalViewer = new SLAMViewer();
         originalViewer.addOctree(map, Color.CORAL, octreeResolution, true);

         originalViewer.addPointCloud(frame2.getCorrectedPointCloudInWorld(), Color.CYAN);
         originalViewer.addOctree(frame2.getFrameMap(), Color.GREEN, surfaceElementResolution, !updateNormal);
         originalViewer.start("originalViewer");
      }

      int numberOfSurfel = frame2.getNumberOfSurfaceElements();
      LogTools.info("numberOfSurfel " + numberOfSurfel);
      Function<DMatrixRMaj, RigidBodyTransform> inputFunction = new Function<DMatrixRMaj, RigidBodyTransform>()
      {
         @Override
         public RigidBodyTransform apply(DMatrixRMaj input)
         {
            RigidBodyTransform transform = new RigidBodyTransform();

            transform.setRotationYawPitchRollAndZeroTranslation(input.get(5), input.get(4), input.get(3));
            transform.getTranslation().set(input.get(0), input.get(1), input.get(2));
            return transform;
         }
      };
      OutputCalculator outputCalculator = new OutputCalculator()
      {
         @Override
         public DMatrixRMaj apply(DMatrixRMaj inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = new RigidBodyTransform(inputFunction.apply(inputParameter));
            RigidBodyTransform correctedLocalPoseInWorld = new RigidBodyTransform(frame2.getUncorrectedLocalPoseInWorld());
            correctedLocalPoseInWorld.multiply(driftCorrectionTransform);

            Plane3D[] correctedSurfel = new Plane3D[numberOfSurfel];
            for (int i = 0; i < numberOfSurfel; i++)
            {
               correctedSurfel[i] = new Plane3D();
               correctedSurfel[i].set(frame2.getSurfaceElementsInLocalFrame().get(i));

               correctedLocalPoseInWorld.transform(correctedSurfel[i].getPoint());
               correctedLocalPoseInWorld.transform(correctedSurfel[i].getNormal());
            }

            DMatrixRMaj errorSpace = new DMatrixRMaj(correctedSurfel.length, 1);
            for (int i = 0; i < correctedSurfel.length; i++)
            {
               double distance = computeClosestDistance(correctedSurfel[i]);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Plane3D surfel)
         {
            return SLAMTools.computeBoundedPerpendicularDistancePointToNormalOctree(map, surfel.getPoint(), map.getResolution());
         }
      };
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(inputFunction, outputCalculator, 6, numberOfSurfel);
      DMatrixRMaj purterbationVector = new DMatrixRMaj(6, 1);
      purterbationVector.set(0, 0.0005);
      purterbationVector.set(1, 0.0005);
      purterbationVector.set(2, 0.0005);
      purterbationVector.set(3, 0.0001);
      purterbationVector.set(4, 0.0001);
      purterbationVector.set(5, 0.0001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(0.05);

      // do ICP.
      double[] qualityArray = new double[50];
      for (int i = 0; i < qualityArray.length; i++)
      {
         qualityArray[i] = optimizer.iterate();
         System.out.println(i + " : " + qualityArray[i]);
      }

      // get parameter.
      RigidBodyTransform icpTransformer = new RigidBodyTransform(inputFunction.apply(optimizer.getOptimalParameter()));
      System.out.println(optimizer.getOptimalParameter());
      frame2.updateOptimizedCorrection(icpTransformer);
      System.out.println("icpTransformer");
      System.out.println(icpTransformer);

      List<? extends Point3DReadOnly> pointCloud = frame2.getCorrectedPointCloudInWorld();
      RigidBodyTransformReadOnly sensorPose = frame2.getCorrectedLocalPoseInWorld();

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = frame2.getCorrectedPointCloudInWorld().size();

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(pointCloud, sensorPose.getTranslation()));

      map.insertScanCollection(scanCollection, false);
      map.enableParallelComputationForNormals(true);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      map.setNormalEstimationParameters(normalEstimationParameters);
      map.updateNormals();

      if (VISUALIZE)
      {
         SLAMViewer slamViewer = new SLAMViewer();
         slamViewer.addOctree(map, Color.CORAL, octreeResolution, true);
         slamViewer.addPointCloud(frame2.getCorrectedPointCloudInWorld(), Color.GREEN);
         slamViewer.addSensorPose(frame2.getCorrectedSensorPoseInWorld(), Color.GREEN);
         slamViewer.addSensorPose(frame2.getUncorrectedSensorPoseInWorld(), Color.BLUE);
         slamViewer.start("testDriftCorrection");

         assertTrue(0.005 > qualityArray[qualityArray.length - 1], "GOOD!");

         ThreadTools.sleepForever();
      }
   }

}
