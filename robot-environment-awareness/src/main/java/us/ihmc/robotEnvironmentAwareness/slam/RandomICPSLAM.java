package us.ihmc.robotEnvironmentAwareness.slam;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.slam.tools.IhmcSLAMTools;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

public class RandomICPSLAM extends IhmcSLAM
{
   public static final boolean DEBUG = false;

   private static final int NUMBER_OF_SOURCE_POINTS = 300;

   private final ConvexPolygon2D previousWindow = new ConvexPolygon2D();
   private static final double WINDOW_WIDTH = 0.4; // 0.6
   private static final double WINDOW_HEIGHT = 0.3; // 0.4
   private static final double WINDOW_HEIGHT_OFFSET = -0.0; // 0.05 is for `testOptimizationSimulattedPointCloud`. 
   private static final double WINDOW_MINIMUM_DEPTH = 0.5;
   private static final double WINDOW_MAXIMUM_DEPTH = 1.5;
   private static final double MINIMUM_OVERLAPPED_RATIO = 0.1;

   private static final double MAXIMUM_INITIAL_DISTANCE_RATIO = 10.0;

   private static final int MAXIMUM_OCTREE_SEARCHING_SIZE = 5;

   private static final boolean USE_WHOLE_OCTREE = true;

   private final NormalOcTree octree;

   // debugging variables.
   public Point3D[] sourcePointsToWorld;
   public Point3D[] correctedSourcePointsToWorld;

   public RandomICPSLAM(double octreeResolution)
   {
      super(octreeResolution);

      octree = new NormalOcTree(octreeResolution);
      previousWindow.addVertex(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2 + WINDOW_HEIGHT_OFFSET);
      previousWindow.addVertex(WINDOW_WIDTH / 2, -WINDOW_HEIGHT / 2 + WINDOW_HEIGHT_OFFSET);
      previousWindow.addVertex(-WINDOW_WIDTH / 2, -WINDOW_HEIGHT / 2 + WINDOW_HEIGHT_OFFSET);
      previousWindow.addVertex(-WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2 + WINDOW_HEIGHT_OFFSET);
      previousWindow.update();
   }

   public NormalOcTree getOctree()
   {
      return octree;
   }

   public void clear()
   {
      super.clear();
      octree.clear();
   }

   @Override
   public RigidBodyTransformReadOnly computeFrameCorrectionTransformer(IhmcSLAMFrame frame)
   {
      IhmcSLAMFrame previousFrame = getLatestFrame();
      Point3DReadOnly[] referencePointCloud = previousFrame.getPointCloud();
      RigidBodyTransformReadOnly referenceSensorPose = previousFrame.getSensorPose();

      // compute NormalOctree for previous frame.
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = referencePointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(IhmcSLAMTools.toScan(referencePointCloud, referenceSensorPose.getTranslation()));

      // using octree from whole previous map would be ideal.
      // Or considering normal vector of the octree would be better.
      if (!USE_WHOLE_OCTREE)
         octree.clear();
      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);

      // if this frame is detected as a key frame, return new RigidBodyTransform();
      // if this frame needs drift correction, return optimized transform;
      // if this frame should not be mergeable, return null;
      // TODO: FB-347: if the angle distance between original sensor pose orientation and new one, think it is key frame.
      // put credit to trust slam. when it exceed, the frame is key frame.

      // see the overlapped area.
      Point3D[] sourcePointsToSensor = IhmcSLAMTools.createSourcePointsToSensorPose(frame, NUMBER_OF_SOURCE_POINTS, previousWindow, WINDOW_MINIMUM_DEPTH,
                                                                                    WINDOW_MAXIMUM_DEPTH, MINIMUM_OVERLAPPED_RATIO);
      if (sourcePointsToSensor == null) // if it is too small overlapped,
      {
         if (DEBUG)
            System.out.println("small overlapped area");
         return new RigidBodyTransform();
      }
      else
      {
         if (DEBUG)
            this.sourcePointsToWorld = IhmcSLAMTools.createConvertedPointsToWorld(frame.getInitialSensorPoseToWorld(), sourcePointsToSensor);

         RigidBodyTransformReadOnly transformWorldToSensorPose = frame.getInitialSensorPoseToWorld();
         SLAMFrameOptimizerCostFunction costFunction = new RandomICPSLAMFrameOptimizerCostFunction(transformWorldToSensorPose, sourcePointsToSensor);

         double initialQuery = costFunction.getQuery(INITIAL_INPUT);
         if (DEBUG)
            System.out.println("frame distance " + initialQuery);

         if (initialQuery > MAXIMUM_INITIAL_DISTANCE_RATIO * getOctreeResolution())
         {
            if (DEBUG)
               System.out.println("too far. will not be merged.");
            return null;
         }
         else
         {
            int numberOfInliers = IhmcSLAMTools.countNumberOfInliers(octree, transformWorldToSensorPose, sourcePointsToSensor, MAXIMUM_OCTREE_SEARCHING_SIZE);
            if (numberOfInliers > 0.99 * sourcePointsToSensor.length)
            {
               if (DEBUG)
                  System.out.println("close enough. many inliers.");
               return new RigidBodyTransform();
            }

            if (DEBUG)
               System.out.println("optimization started. " + initialQuery);
            GradientDescentModule optimizer = new GradientDescentModule(costFunction, INITIAL_INPUT);

            int maxIterations = 300;
            double convergenceThreshold = 1 * 10E-5;
            double optimizerStepSize = -1.0;
            double optimizerPerturbationSize = 0.00001;

            optimizer.setInputLowerLimit(LOWER_LIMIT);
            optimizer.setInputUpperLimit(UPPER_LIMIT);
            optimizer.setMaximumIterations(maxIterations);
            optimizer.setConvergenceThreshold(convergenceThreshold);
            optimizer.setStepSize(optimizerStepSize);
            optimizer.setPerturbationSize(optimizerPerturbationSize);
            optimizer.setReducingStepSizeRatio(2);

            int run = optimizer.run();
            if (DEBUG)
               System.out.println("optimization result # [" + run + "], #" + optimizer.getComputationTime() + " sec # " + "Init Q: " + initialQuery
                     + ", Opt Q: " + optimizer.getOptimalQuery());
            TDoubleArrayList optimalInput = optimizer.getOptimalInput();

            if (DEBUG)
               System.out.println("optimalInput # " + optimalInput.get(0) + " " + optimalInput.get(1) + " " + optimalInput.get(2));

            RigidBodyTransform transformer = new RigidBodyTransform();
            costFunction.convertToSensorPoseMultiplier(optimalInput, transformer);

            if (DEBUG)
            {
               RigidBodyTransform optimizedSensorPose = new RigidBodyTransform(transformWorldToSensorPose);
               optimizedSensorPose.multiply(transformer);
               correctedSourcePointsToWorld = IhmcSLAMTools.createConvertedPointsToWorld(optimizedSensorPose, sourcePointsToSensor);
            }

            if (DEBUG)
               System.out.println(transformer);
            return transformer;
         }
      }
   }

   class RandomICPSLAMFrameOptimizerCostFunction extends SLAMFrameOptimizerCostFunction
   {
      final Point3DReadOnly[] sourcePointsToSensor;

      RandomICPSLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose, Point3DReadOnly[] sourcePointsToSensor)
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
         int numberOfInliers = 0; // TODO: will be used in future.
         int numberOfOutliers = 0;
         Point3D newSourcePointToWorld = new Point3D();
         for (Point3DReadOnly sourcePoint : sourcePointsToSensor)
         {
            newSourcePointToWorld.set(sourcePoint);
            newSensorPose.transform(newSourcePointToWorld);

            double distance = IhmcSLAMTools.computeDistanceToNormalOctree(octree, newSourcePointToWorld, MAXIMUM_OCTREE_SEARCHING_SIZE);

            if (distance < 0)
            {
               distance = MAXIMUM_OCTREE_SEARCHING_SIZE * getOctreeResolution();
            }

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

         return totalDistance / sourcePointsToSensor.length;
      }
   }
}
