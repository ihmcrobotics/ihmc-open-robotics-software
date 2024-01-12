package us.ihmc.perception;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape.BOX;

public class IterativeClosestPointWorker
{
   private static final double discountFactor = 0.9;

   private static final boolean ignoreShapeTypeWhenSegmenting = false;
   private static final float defaultXLength = 0.2f;
   private static final float defaultYLength = 0.4f;
   private static final float defaultZLength = 0.3f;
   private static final float defaultXRadius = 0.1f;
   private static final float defaultYRadius = 0.1f;
   private static final float defaultZRadius = 0.1f;
   private static final PrimitiveRigidBodyShape defaultDetectionShape = BOX;
   private static boolean useParallelStreams = false;

   private final Random random;

   private long sceneNodeID = -1L;

   private PrimitiveRigidBodyShape detectionShape;
   private float xLength = 0.19f;
   private float yLength = 0.4f;
   private float zLength = 0.31f;
   private float xRadius = 0.1f;
   private float yRadius = 0.1f;
   private float zRadius = 0.1f;
   private int numberOfCorrespondences;

   private final DMatrixRMaj objectRelativeToCentroidPoints;
   private final DMatrixRMaj measurementRelativeToCentroidPoints;

   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);

   private List<? extends Point3DReadOnly> measurementPointCloud;
   private final Object measurementPointCloudSynchronizer = new Object();

   private List<Point3DReadOnly> segmentedPointCloud;
   private double segmentSphereRadius = 1.0;

   private List<? extends Point3DReadOnly> neighborPointCloud;

   private List<Point3DReadOnly> correspondingObjectPoints;
   private List<Point3DReadOnly> correspondingMeasurementPoints;

   private List<Point3D32> localObjectPoints;
   private List<Point3D32> objectInWorldPoints;
   private boolean objectInWorldPointsIsUpToDate = false;

   private final AtomicBoolean useTargetPoint = new AtomicBoolean(true);
   // Point around which ICP will segment the measurement point cloud
   private final Point3D targetPoint = new Point3D();

   private final Pose3D resultPose = new Pose3D();

   public IterativeClosestPointWorker(int numberOfObjectSamples, int numberOfCorrespondences, Random random)
   {
      this(defaultDetectionShape,
           defaultXLength,
           defaultYLength,
           defaultZLength,
           defaultXRadius,
           defaultYRadius,
           defaultZRadius,
           numberOfObjectSamples,
           numberOfCorrespondences,
           new Pose3D(),
           random);
   }

   public IterativeClosestPointWorker(PrimitiveRigidBodyShape objectShape,
                                      float xLength,
                                      float yLength,
                                      float zLength,
                                      float xRadius,
                                      float yRadius,
                                      float zRadius,
                                      int numberOfObjectSamples,
                                      int numberOfCorrespondences,
                                      Pose3DReadOnly initialPose,
                                      Random random)
   {
      detectionShape = objectShape;
      this.xLength = xLength;
      this.yLength = yLength;
      this.zLength = zLength;
      this.xRadius = xRadius;
      this.yRadius = yRadius;
      this.zRadius = zRadius;
      this.numberOfCorrespondences = numberOfCorrespondences;
      this.random = random;

      targetPoint.set(initialPose.getPosition());

      objectRelativeToCentroidPoints = new DMatrixRMaj(numberOfCorrespondences, 3);
      measurementRelativeToCentroidPoints = new DMatrixRMaj(numberOfCorrespondences, 3);

      localObjectPoints = IterativeClosestPointTools.createICPObjectPointCloud(detectionShape,
                                                                               new Pose3D(),
                                                                               xLength,
                                                                               yLength,
                                                                               zLength,
                                                                               xRadius,
                                                                               yRadius,
                                                                               zRadius,
                                                                               numberOfObjectSamples,
                                                                               random);
      setPoseGuess(initialPose);
   }

   public boolean runICP(int numberOfIterations)
   {
      // Determine around where the point cloud should be segmented
      // (user provided point or last centroid)
      Pose3D detectionPoint = new Pose3D();
      if (useTargetPoint.get())
         detectionPoint.getPosition().set(targetPoint);
      else
         detectionPoint.set(resultPose);


      boolean ranICPSuccessfully = false;
      for (int i = 0; i < numberOfIterations; ++i)
      {
         // Segment the point cloud
         synchronized (measurementPointCloudSynchronizer) // synchronize as to avoid changes to environment point cloud while segmenting it
         {
            if (measurementPointCloud == null)
               return false;

            // TODO: find way to make shape segmentation not suck ICP into ground (without side effects)
            if (numberOfIterations > 1 && i == 0)  // Running multiple iterations, on first iteration segment & find neighbors
               segmentPointCloudAndFindNeighbors(measurementPointCloud, detectionPoint, segmentSphereRadius);
            else if (numberOfIterations > 1)       // Running multiple iterations, on following iterations use neighbor points for segmentation (faster)
               segmentPointCloud(neighborPointCloud, detectionPoint, segmentSphereRadius);
            else                                   // Running only one iteration, don't bother finding neighbors
               segmentPointCloud(measurementPointCloud, detectionPoint, segmentSphereRadius);
         }

         // Only run ICP iteration if segmented point cloud has enough points
         if (segmentedPointCloud.size() >= numberOfCorrespondences)
         {
            runICPIteration();

            ranICPSuccessfully = true;
         }
      }

      return ranICPSuccessfully;
   }

   public DetectedObjectPacket getResult()
   {
      DetectedObjectPacket resultMessage = new DetectedObjectPacket();
      resultMessage.setId((int) sceneNodeID);
      resultMessage.getPose().set(resultPose);
      for (Point3D32 point3D32 : getObjectPointCloud())
         resultMessage.getObjectPointCloud().add().set(point3D32);
      for (int i = 0; i < segmentedPointCloud.size(); i++)
         resultMessage.getSegmentedPointCloud().add().set(segmentedPointCloud.get(i));

      return resultMessage;
   }

   // TODO: Pass in a Pose to transform (all object points define WRT the pose of object)
   private void runICPIteration()
   {
      Collections.shuffle(segmentedPointCloud, random);

      // Calculate nearest neighbor for each point (environment to object)
      correspondingObjectPoints = new ArrayList<>();
      correspondingMeasurementPoints = new ArrayList<>();

      if (IterativeClosestPointTools.canComputeCorrespondencesOnShape(detectionShape))
      {
         IterativeClosestPointTools.computeCorrespondencesOnShape(detectionShape,
                                                                  resultPose,
                                                                  segmentedPointCloud,
                                                                  correspondingMeasurementPoints,
                                                                  correspondingObjectPoints,
                                                                  xLength,
                                                                  yLength,
                                                                  zLength,
                                                                  xRadius,
                                                                  yRadius,
                                                                  zRadius,
                                                                  numberOfCorrespondences);
      }
      else
      {
         computeCorrespondingPointsBetweenMeasurementAndObjectPointCloud(segmentedPointCloud,
                                                                         correspondingMeasurementPoints,
                                                                         correspondingObjectPoints);
      }

      // Calculate object corresponce centroid
      Point3D32 objectCentroid = computeCentroidOfPointCloud(correspondingObjectPoints);
      int foundCorrespondences = correspondingMeasurementPoints.size();

      for (int i = 0; i < foundCorrespondences; ++i)
      {
         objectRelativeToCentroidPoints.set(i, 0, correspondingObjectPoints.get(i).getX() - objectCentroid.getX());
         objectRelativeToCentroidPoints.set(i, 1, correspondingObjectPoints.get(i).getY() - objectCentroid.getY());
         objectRelativeToCentroidPoints.set(i, 2, correspondingObjectPoints.get(i).getZ() - objectCentroid.getZ());
      }

      // Calculate measurement centroid
      Point3D32 measurementCentroid = computeCentroidOfPointCloud(correspondingMeasurementPoints);

      for (int i = 0; i < foundCorrespondences; ++i)
      {
         measurementRelativeToCentroidPoints.set(i, 0, correspondingMeasurementPoints.get(i).getX() - measurementCentroid.getX());
         measurementRelativeToCentroidPoints.set(i, 1, correspondingMeasurementPoints.get(i).getY() - measurementCentroid.getY());
         measurementRelativeToCentroidPoints.set(i, 2, correspondingMeasurementPoints.get(i).getZ() - measurementCentroid.getZ());
      }

      // Initialize matrix variables
      DMatrixRMaj H = new DMatrixRMaj(3, 3);
      DMatrixRMaj U = new DMatrixRMaj(3, 3);
      DMatrixRMaj V = new DMatrixRMaj(3, 3);
      RotationMatrix optimalRotationMatrix = new RotationMatrix();
      DMatrixRMaj optimalRotation = new DMatrixRMaj(3, 3); // This is the optimized rotation matrix to rotate the measurement to match the point cloud.
      Point3D32 objectAdjustedLocation = new Point3D32();
      Point3D32 objectTranslation = new Point3D32();

      // TODO: Get new translation every iteration, append to last, modify object pose using the translation
      // Solve for Best Fit Transformation
      CommonOps_DDRM.multTransA(objectRelativeToCentroidPoints, measurementRelativeToCentroidPoints, H);
      svdSolver.decompose(H);
      svdSolver.getU(U, false);
      svdSolver.getV(V, true);
      CommonOps_DDRM.multTransAB(V, U, optimalRotation);

      // Correct any negative rotation matrices which come out of SVD its positive counterpart (fixes NotARotationMatrix issues)
      if (CommonOps_DDRM.det(optimalRotation) < 0.0)
         CommonOps_DDRM.scale(-1.0, optimalRotation);

      /* Calculate object transform */
      // Create the transform, and set the rotation, so it's a pure rotation transform
      optimalRotationMatrix.set(optimalRotation);
      RigidBodyTransform objectToMeasurementTransform = new RigidBodyTransform();
      objectToMeasurementTransform.getRotation().interpolate(optimalRotationMatrix, discountFactor);

      // Calcualte the necessary translation
      objectToMeasurementTransform.transform(objectCentroid, objectAdjustedLocation);
      objectTranslation.sub(measurementCentroid, objectAdjustedLocation);

      // set that translation into the transform
      objectToMeasurementTransform.getTranslation().setAndScale(discountFactor, objectTranslation);

      // Rotate and translate the resulting pose according to the correction transform.
      resultPose.applyTransform(objectToMeasurementTransform);
      objectInWorldPointsIsUpToDate = false;
   }

   private void segmentPointCloud(List<? extends Point3DReadOnly> measurementPointCloud, Pose3DReadOnly virtualObjectPointInWorld, double cutoffRange)
   {
      double cutoffSquare = MathTools.square(cutoffRange);

      Stream<? extends Point3DReadOnly> measurementStream = useParallelStreams ? measurementPointCloud.parallelStream() : measurementPointCloud.stream();

      segmentedPointCloud = measurementStream.filter(point -> IterativeClosestPointTools.distanceSquaredFromShape(detectionShape,
                                                                                                                  virtualObjectPointInWorld,
                                                                                                                  point,
                                                                                                                  xLength,
                                                                                                                  yLength,
                                                                                                                  zLength,
                                                                                                                  xRadius,
                                                                                                                  yRadius,
                                                                                                                  zRadius,
                                                                                                                  ignoreShapeTypeWhenSegmenting)
                                                              <= cutoffSquare).collect(Collectors.toList());
   }

   private void segmentPointCloudAndFindNeighbors(List<? extends Point3DReadOnly> measurementPointCloud, Pose3DReadOnly virtualObjectPointInWorld, double cutoffRange)
   {
      // The below two lists should be synchronized to allow use of parallel streams
      segmentedPointCloud = Collections.synchronizedList(new ArrayList<>());
      neighborPointCloud.clear();

      Stream<? extends Point3DReadOnly> measurementStream = useParallelStreams ? measurementPointCloud.parallelStream() : measurementPointCloud.stream();
      List<DistancedPoint> distancedPoints = measurementStream.map(point ->
                                                                   {
                                                                      double distance = IterativeClosestPointTools.distanceSquaredFromShape(detectionShape,
                                                                                                                                            virtualObjectPointInWorld,
                                                                                                                                            point,
                                                                                                                                            xLength,
                                                                                                                                            yLength,
                                                                                                                                            zLength,
                                                                                                                                            xRadius,
                                                                                                                                            yRadius,
                                                                                                                                            zRadius,
                                                                                                                                            ignoreShapeTypeWhenSegmenting);
                                                                      return new DistancedPoint(point, distance);
                                                                   }).toList();

      segmentPointCloudUsingSphereAndFindNeighbors(distancedPoints, cutoffRange);
   }

   public void segmentPointCloudUsingSphere(List<Point3DReadOnly> measurementPointCloud, Pose3DReadOnly virtualObjectPointInWorld, double cutoffRange)
   {
      double cutoffRangeSquared = cutoffRange * cutoffRange;
      Stream<Point3DReadOnly> measurementStream = useParallelStreams ? measurementPointCloud.parallelStream() : measurementPointCloud.stream();
      segmentedPointCloud = measurementStream.filter(point -> point.distanceSquared(virtualObjectPointInWorld.getPosition()) < cutoffRangeSquared)
                                             .collect(Collectors.toList());
   }

   public void segmentPointCloudUsingSphereAndFindNeighbors(List<Point3DReadOnly> measurementPointCloud, Pose3DReadOnly virtualObjectPointInWorld, double cutoffRange)
   {
      Stream<Point3DReadOnly> measurementStream = useParallelStreams ? measurementPointCloud.parallelStream() : measurementPointCloud.stream();
      List<DistancedPoint> distancedPoints = measurementStream.map(point -> new DistancedPoint(point, point.distanceSquared(virtualObjectPointInWorld.getPosition()))).toList();

      segmentPointCloudUsingSphereAndFindNeighbors(distancedPoints, cutoffRange);
   }

   public void segmentPointCloudUsingSphereAndFindNeighbors(List<DistancedPoint> distancedPoints, double cutoffRange)
   {
      double neighborCutoff = 2.0 * cutoffRange;
      double cutoffRangeSquared = cutoffRange * cutoffRange;
      double neighborCutoffRangeSquared = neighborCutoff * neighborCutoff;

      Stream<DistancedPoint> distancedPointsStream = useParallelStreams ? distancedPoints.parallelStream() : distancedPoints.stream();
      List<DistancedPoint> neighborPointCloud = distancedPointsStream.filter(point -> point.getDistanceSquared() < neighborCutoffRangeSquared).collect(Collectors.toList());
      this.neighborPointCloud = neighborPointCloud;

      Stream<DistancedPoint> neighborPointsStream = useParallelStreams ? neighborPointCloud.parallelStream() : neighborPointCloud.stream();
      segmentedPointCloud = neighborPointsStream.filter(point -> point.getDistanceSquared() < cutoffRangeSquared).collect(Collectors.toList());
   }

   private static class DistancedPoint extends Point3D32
   {
      private final double distanceSquared;

      public DistancedPoint(Point3DReadOnly point, double distanceSquared)
      {
         super(point);
         this.distanceSquared = distanceSquared;
      }

      public double getDistanceSquared()
      {
         return distanceSquared;
      }
   }

   private void computeCorrespondingPointsBetweenMeasurementAndObjectPointCloud(List<Point3DReadOnly> measurementPoints,
                                                                                List<Point3DReadOnly> correspondingMeasurementPointsToPack,
                                                                                List<Point3DReadOnly> correspondingObjectPointsToPack)
   {
      int measurementIdx = 0;
      int iteration = 0;
      List<Point3D32> objectInWorldPoints = getObjectPointCloud();
      int maxIterations = 2 * objectInWorldPoints.size();

      while (correspondingMeasurementPointsToPack.size() < numberOfCorrespondences && measurementIdx < measurementPoints.size() && iteration < maxIterations)
      {
         Point3DReadOnly measurementPoint = measurementPoints.get(measurementIdx++);
         double minDistance = Double.POSITIVE_INFINITY;
         Point3D32 correspondingObjectPoint = null;

         for (Point3D32 objectPoint : objectInWorldPoints)
         {
            double distanceSquared = measurementPoint.distanceSquared(objectPoint);
            if (distanceSquared < minDistance)
            {
               minDistance = distanceSquared;
               correspondingObjectPoint = objectPoint;
            }
         }
         if (correspondingObjectPoint == null)
            continue;

         // record
         correspondingMeasurementPointsToPack.add(measurementPoint);
         correspondingObjectPointsToPack.add(correspondingObjectPoint);

         iteration++;
      }
   }

   private static Point3D32 computeCentroidOfPointCloud(List<Point3DReadOnly> pointCloud)
   {
      return computeCentroidOfPointCloud(pointCloud, pointCloud.size());
   }

   private static Point3D32 computeCentroidOfPointCloud(List<Point3DReadOnly> pointCloud, int pointsToAverage)
   {
      Point3D32 centroid = new Point3D32();
      for (int i = 0; i < pointsToAverage; i++)
         centroid.add(pointCloud.get(i));
      centroid.scale(1.0 / pointsToAverage);

      return centroid;
   }

   // TODO: Color filtering could go here
   // pass in depth and color image -> cut out invalid pixels of depth image based on color image ->
   // -> create "environmentPointCloud" based off the color-filtered depth image ->
   // -> set the segmentation radius to be large (1.0~1.5?)
   public void setEnvironmentPointCloud(List<? extends Point3DReadOnly> pointCloud)
   {
      synchronized (measurementPointCloudSynchronizer)
      {
         measurementPointCloud = pointCloud;
      }
   }

   public void setSegmentSphereRadius(double radius)
   {
      segmentSphereRadius = radius;
   }

   public void useProvidedTargetPoint(boolean targetPointOn)
   {
      useTargetPoint.set(targetPointOn);
   }

   public void setTargetPoint(Point3DBasics detectionPoint)
   {
      targetPoint.set(detectionPoint);
   }

   public void setSceneNodeID(long id)
   {
      sceneNodeID = id;
   }

   public void setNumberOfCorrespondences(int numberOfCorrespondences)
   {
      this.numberOfCorrespondences = numberOfCorrespondences;
      objectRelativeToCentroidPoints.reshape(numberOfCorrespondences, 3);
      measurementRelativeToCentroidPoints.reshape(numberOfCorrespondences, 3);
   }

   public void setDetectionShape(PrimitiveRigidBodyShape shape)
   {
      detectionShape = shape;
      changeSize(xLength, yLength, zLength, xRadius, yRadius, zRadius, localObjectPoints.size());
   }

   public void changeSize(float xLength, float yLength, float zLength, float xRadius, float yRadius, float zRadius, int numberOfObjectSamples)
   {
      this.xLength = xLength;
      this.yLength = yLength;
      this.zLength = zLength;
      this.xRadius = xRadius;
      this.yRadius = yRadius;
      this.zRadius = zRadius;

      localObjectPoints = IterativeClosestPointTools.createICPObjectPointCloud(detectionShape,
                                                                               new Pose3D(),
                                                                               xLength,
                                                                               yLength,
                                                                               zLength,
                                                                               xRadius,
                                                                               yRadius,
                                                                               zRadius,
                                                                               numberOfObjectSamples,
                                                                               random);
      setPoseGuess(resultPose);
   }

   public List<Point3DReadOnly> getSegmentedPointCloud()
   {
      return segmentedPointCloud;
   }

   public List<Point3DReadOnly> getCorrespondingMeasurementPoints()
   {
      return correspondingMeasurementPoints;
   }

   public List<Point3DReadOnly> getCorrespondingObjectPoints()
   {
      return correspondingObjectPoints;
   }

   public void setPoseGuess(Pose3DReadOnly pose)
   {
      resultPose.set(pose);
      objectInWorldPointsIsUpToDate = false;
      //      updateObjectInWorldPoints();
   }

   private void updateObjectInWorldPoints()
   {
      if (objectInWorldPointsIsUpToDate)
         return;

      Stream<Point3D32> localPointsStream = useParallelStreams ? localObjectPoints.parallelStream() : localObjectPoints.stream();
      objectInWorldPoints = localPointsStream.map(localPoint ->
                                                  {
                                                     Point3D32 pointInWorld = new Point3D32(localPoint);
                                                     resultPose.transform(pointInWorld);
                                                     return pointInWorld;
                                                  }).collect(Collectors.toList());
      objectInWorldPointsIsUpToDate = true;
   }

   public Pose3DReadOnly getResultPose()
   {
      return resultPose;
   }

   public List<Point3D32> getObjectPointCloud()
   {
      if (!objectInWorldPointsIsUpToDate)
         updateObjectInWorldPoints();
      return objectInWorldPoints;
   }

   public float getXLength()
   {
      return xLength;
   }

   public float getYLength()
   {
      return yLength;
   }

   public float getZLength()
   {
      return zLength;
   }

   public float getXRadius()
   {
      return xRadius;
   }

   public float getYRadius()
   {
      return yRadius;
   }

   public float getZRadius()
   {
      return zRadius;
   }

   public boolean isUsingTargetPoint()
   {
      return useTargetPoint.get();
   }
}
