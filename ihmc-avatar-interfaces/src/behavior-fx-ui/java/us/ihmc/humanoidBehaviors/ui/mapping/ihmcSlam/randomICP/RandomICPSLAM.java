package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.randomICP;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.IhmcSLAMTools;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAM;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAMFrame;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrameOptimizerCostFunction;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

public class RandomICPSLAM extends IhmcSLAM
{
   public final List<Point3D> sourcePointsToWorld = new ArrayList<>();

   private static final int numberOfSourcePoints = 300;

   private final ConvexPolygon2D previousWindow = new ConvexPolygon2D();
   private static final double windowWidth = 0.6;
   private static final double windowHeight = 0.4;
   private static final double windowDepthThreshold = 0.5;
   private static final double minimumOverlappedRatio = 0.1;

   private static final int maximumSearchingSize = 5;

   private final NormalOcTree octree;

   public RandomICPSLAM(double octreeResolution)
   {
      super(octreeResolution);

      octree = new NormalOcTree(octreeResolution);
      previousWindow.addVertex(windowWidth / 2, windowHeight / 2);
      previousWindow.addVertex(windowWidth / 2, -windowHeight / 2);
      previousWindow.addVertex(-windowWidth / 2, -windowHeight / 2);
      previousWindow.addVertex(-windowWidth / 2, windowHeight / 2);
      previousWindow.update();
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

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);

      octree.updateNormals();

      // if this frame is detected as a key frame, return new RigidBodyTransform();
      // if this frame needs drift correction, return optimized transform;
      // if this frame should not be mergeable, return null;

      // see the overlapped area.
      Point3D[] sourcePointsToSensor = IhmcSLAMTools.createSourcePointsToSensorPose(frame, numberOfSourcePoints, previousWindow, windowDepthThreshold,
                                                                                    minimumOverlappedRatio);
      if (sourcePointsToSensor == null) // if it is too small overlapped,
      {
         System.out.println("small overlapped area");
         return new RigidBodyTransform();
      }
      else
      {
         RigidBodyTransformReadOnly transformWorldToSensorPose = frame.getInitialSensorPoseToWorld();
         SLAMFrameOptimizerCostFunction costFunction = new RICPSLAMFrameOptimizerCostFunction(transformWorldToSensorPose, sourcePointsToSensor);

         double initialQuery = costFunction.getQuery(INITIAL_QUERY);
         System.out.println("frame distance " + initialQuery);

         // TODO: if the source points are too far.
         if (initialQuery > 2 * getOctreeResolution())
         {
            System.out.println("too far. will not be merged.");
            return null;
         }
         else
         {
            int numberOfInliers = IhmcSLAMTools.countNumberOfInliers(octree, transformWorldToSensorPose, sourcePointsToSensor, maximumSearchingSize);
            if(numberOfInliers > 0.9 * sourcePointsToSensor.length)
            {
               System.out.println("close enough. many inliers.");
               return new RigidBodyTransform();   
            }
            
            System.out.println("optimization started. " + initialQuery);
            //            return new RigidBodyTransform();
            GradientDescentModule optimizer = new GradientDescentModule(costFunction, INITIAL_QUERY);

            int maxIterations = 100;
            double convergenceThreshold = 1 * 10E-4;
            double optimizerStepSize = -1.0;
            double optimizerPerturbationSize = 0.001;

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

//          System.out.println();
//          System.out.println(transformer);

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
         double totalOutliersDistance = 0;
         int numberOfInliers = 0;
         int numberOfOutliers = 0;
         Point3D newSourcePointToWorld = new Point3D();
         for (Point3DReadOnly sourcePoint : sourcePointsToSensor)
         {
            newSourcePointToWorld.set(sourcePoint);
            newSensorPose.transform(newSourcePointToWorld);

            double distance = IhmcSLAMTools.computeDistanceToNormalOctree2(octree, newSourcePointToWorld, maximumSearchingSize);

            if (distance >= 0)
            {
               totalDistance = totalDistance + distance;

               if (distance > octree.getResolution())
               {
                  totalOutliersDistance = totalOutliersDistance + distance;
                  numberOfOutliers++;
               }
               else
               {
                  numberOfInliers++;
               }
            }
         }

         if (numberOfInliers == 0)
            return 100;

         return totalOutliersDistance / numberOfOutliers;
      }
   }
}
