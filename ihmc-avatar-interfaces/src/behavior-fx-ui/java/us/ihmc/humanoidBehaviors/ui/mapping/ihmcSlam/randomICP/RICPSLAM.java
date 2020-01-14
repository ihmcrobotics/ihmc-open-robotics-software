package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.randomICP;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.IhmcSLAMTools;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAM;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAMFrame;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrameOptimizerCostFunction;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

public class RICPSLAM extends IhmcSLAM
{
   public final List<Point3D> sourcePointsToWorld = new ArrayList<>();
   
   private final Point3D[] sourcePointsToSensor;
   
   private static final int numberOfSourcePoints = 300;
   private static final Random randomSelector = new Random(0612L);

   private static final double watchingWindowWidth = 1.0;
   private static final double watchingWindowHeight = 0.3;
   private static final double watchingWindowHeightOffset = 0.0;
   private static final double watchingDepthThreshold = 0.5;

   private static final int initialSearchingSize = 3;
   private static final int maximumSearchingSize = 20;

   private final NormalOcTree octree;

   public RICPSLAM(double octreeResolution)
   {
      super(octreeResolution);

      octree = new NormalOcTree(octreeResolution);
      sourcePointsToSensor = new Point3D[numberOfSourcePoints];
   }

   @Override
   protected RigidBodyTransformReadOnly computeFrameCorrectionTransformer(IhmcSLAMFrame frame)
   {
      IhmcSLAMFrame previousFrame = getLatestFrame();
      Point3DReadOnly[] referencePointCloud = previousFrame.getPointCloud();
      RigidBodyTransformReadOnly referenceSensorPose = previousFrame.getSensorPose();

      // compute NormalOctree for previous frame.
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = referencePointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(IhmcSLAMTools.toScan(referencePointCloud, referenceSensorPose.getTranslation()));

      // TODO: using octree from whole previous map would be ideal.
      // Or considering normal vector of the octree would be better.
      octree.clear();
      octree.insertScanCollection(scanCollection, false);

      // if this frame is detected as a key frame, return new RigidBodyTransform();
      // if this frame needs drift correction, return optimized transform;
      // if this frame should not be mergeable, return null;

      // TODO: see the overlapped area.
      if (false) // if it is too small,
      {
         return new RigidBodyTransform();
      }
      else
      {
         selectSourcePointsToSensor(frame);
         RigidBodyTransformReadOnly transformWorldToSensorPose = frame.getInitialSensorPoseToWorld();
         SLAMFrameOptimizerCostFunction costFunction = new RICPSLAMFrameOptimizerCostFunction(transformWorldToSensorPose, sourcePointsToSensor);

         double initialQuery = costFunction.getQuery(INITIAL_QUERY);

         // TODO: if the source points are too far.
         if (false)
         {
            return null;
         }
         else
         {
            GradientDescentModule optimizer = new GradientDescentModule(costFunction, INITIAL_QUERY);

            int maxIterations = 100;
            double convergenceThreshold = 1 * 10E-4;
            double optimizerStepSize = -1.0;
            double optimizerPerturbationSize = 0.005;

            optimizer.setInputLowerLimit(LOWER_LIMIT);
            optimizer.setInputUpperLimit(UPPER_LIMIT);
            optimizer.setMaximumIterations(maxIterations);
            optimizer.setConvergenceThreshold(convergenceThreshold);
            optimizer.setStepSize(optimizerStepSize);
            optimizer.setPerturbationSize(optimizerPerturbationSize);
            optimizer.setReducingStepSizeRatio(5);

            int run = optimizer.run();
            System.out.println("optimization result # " + run + "              " + initialQuery + " " + optimizer.getOptimalQuery());
            TDoubleArrayList optimalInput = optimizer.getOptimalInput();

            RigidBodyTransform transformer = new RigidBodyTransform();
            costFunction.convertToSensorPoseMultiplier(optimalInput, transformer);

            return transformer;
         }
      }
   }

   class RICPSLAMFrameOptimizerCostFunction extends SLAMFrameOptimizerCostFunction
   {
      final Point3DReadOnly[] sourcePointsToSensor;

      RICPSLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose, Point3DReadOnly[] sourcePointsToSensor)
      {
         super(transformWorldToSensorPose);
         this.sourcePointsToSensor = sourcePointsToSensor;
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
         for (Point3DReadOnly sourcePoint : sourcePointsToSensor)
         {
            newSourcePointToWorld.set(sourcePoint);
            newSensorPose.transform(newSourcePointToWorld);

            int searchingSize = initialSearchingSize;
            double distance = -1.0;
            while (distance < 0 && searchingSize < maximumSearchingSize)
            {
               distance = IhmcSLAMTools.computeDistanceToNormalOctree(octree, newSourcePointToWorld, searchingSize);
               
//               IhmcSLAMFrame previousFrame = getLatestFrame();
//               Point3DReadOnly[] referencePointCloud = previousFrame.getPointCloud();
//               distance = IhmcSLAMTools.computeDistanceToPointCloud(referencePointCloud, newSourcePointToWorld);
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

   private void selectSourcePointsToSensor(IhmcSLAMFrame frame)
   {
      Point3DReadOnly[] pointCloudToSensorPose = frame.getOriginalPointCloudToSensorPose();

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
                  if (-watchingWindowHeight / 2 + watchingWindowHeightOffset < selectedPoint.getY()
                        && selectedPoint.getY() < watchingWindowHeight / 2 + watchingWindowHeightOffset)
                  {
                     indexOfSourcePoints.add(selectedIndex);
                  }
               }
            }
         }

         for (int i = 0; i < indexOfSourcePoints.size(); i++)
         {
            sourcePointsToSensor[i] = new Point3D(pointCloudToSensorPose[indexOfSourcePoints.get(i)]);
            sourcePointsToWorld.add(new Point3D(frame.getOriginalPointCloud()[indexOfSourcePoints.get(i)]));
         }
      }
   }
}
