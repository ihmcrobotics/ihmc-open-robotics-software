package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.randomICP;

import java.util.Random;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.IhmcSLAMTools;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrame;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrameOptimizerCostFunction;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;

public class RandomICPSLAMFrameOptimizerCostFunction extends SLAMFrameOptimizerCostFunction
{
   private static final int numberOfSourcePoints = 100;
   private static final Random randomSelector = new Random(0612L);

   private static final double watchingWindowWidth = 0.5;
   private static final double watchingWindowHeight = 0.5;
   private static final double watchingDepthThreshold = 0.5;

   private static final int initialSearchingSize = 3;
   private static final int maximumSearchingSize = 20;

   private final NormalOcTree octree;

   private final Point3DReadOnly[] sourcePointsToSensor = new Point3D[numberOfSourcePoints];

   public RandomICPSLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose, Point3DReadOnly[] pointCloudToSensorPose,
                                                  SLAMFrame previousFrame, double octreeResolution)
   {
      super(transformWorldToSensorPose);

      Point3DReadOnly[] referencePointCloud = previousFrame.getPointCloud();
      RigidBodyTransformReadOnly referenceSensorPose = previousFrame.getSensorPose();

      // compute NormalOctree for previous frame.
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = referencePointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(IhmcSLAMTools.toScan(referencePointCloud, referenceSensorPose.getTranslation()));

      octree = new NormalOcTree(octreeResolution);

      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);

      // Select source points.
      TIntArrayList indexOfSourcePoints = new TIntArrayList();
      while (indexOfSourcePoints.size() != numberOfSourcePoints)
      {
         int selectedIndex = randomSelector.nextInt(pointCloudToSensorPose.length);
         if (!indexOfSourcePoints.contains(selectedIndex))
         {
            Point3DReadOnly selectedPoint = pointCloudToSensorPose[selectedIndex];
            if (selectedPoint.getZ() > watchingDepthThreshold)
            {
               if (-watchingWindowWidth / 2 < selectedPoint.getX() && selectedPoint.getX() < watchingWindowWidth / 2)
               {
                  if (-watchingWindowHeight / 2 < selectedPoint.getY() && selectedPoint.getY() < watchingWindowHeight / 2)
                  {
                     indexOfSourcePoints.add(selectedIndex);
                  }
               }
            }
         }

         for (int i = 0; i < indexOfSourcePoints.size(); i++)
         {
            sourcePointsToSensor[i] = new Point3D(pointCloudToSensorPose[indexOfSourcePoints.get(i)]);
         }
      }
   }

   @Override
   public double getQuery(TDoubleArrayList values)
   {
      /**
       * values are difference in 6 dimensions : dx, dy, dz, du, dv, dw
       */
      RigidBodyTransform sensorPoseMultiplier = new RigidBodyTransform();
      convertToSensorPoseMultiplier(values, sensorPoseMultiplier);

      RigidBodyTransform newSensorPose = new RigidBodyTransform(transformWorldToSensorPose);
      newSensorPose.multiply(sensorPoseMultiplier);

      double totalDistance = 0;
      int numberOfInliers = 0;
      Point3D newSourcePointToWorld = new Point3D();
      int index = 0;
      for (Point3DReadOnly sourcePoint : sourcePointsToSensor)
      {
         newSourcePointToWorld.set(sourcePoint);
         newSensorPose.transform(newSourcePointToWorld);

         staticPoints[index].set(newSourcePointToWorld);
         index++;

         int searchingSize = initialSearchingSize;
         double distance = -1.0;
         while (distance < 0 && searchingSize < maximumSearchingSize)
         {
            distance = IhmcSLAMTools.computeDistanceToNormalOctree(octree, newSourcePointToWorld, searchingSize);
            searchingSize++;
         }

         if (distance > 0)
         {
            totalDistance = totalDistance + distance * distance;
            numberOfInliers++;
         }
      }

      return totalDistance / numberOfInliers; // handle if the numberOfInliers are low.
   }
}
