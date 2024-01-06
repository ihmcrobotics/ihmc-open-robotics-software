package us.ihmc.perception;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BinaryOperator;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class IterativeClosestPointWorker
{
   private final Random random;

   private final ROS2Helper ros2Helper;
   private long sceneNodeID = -1L;

   private final PrimitiveRigidBodyShape detectionShape;
   private float xLength = 0.19f;
   private float yLength = 0.4f;
   private float zLength = 0.31f;
   private float xRadius = 0.1f;
   private float yRadius = 0.1f;
   private float zRadius = 0.1f;
   private int numberOfICPObjectPoints;

   private boolean useParallelStreams = false;


   private final DMatrixRMaj objectRelativeToCentroidPoints;
   private final DMatrixRMaj measurementRelativeToCentroidPoints;


   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);

   private List<Point3D32> measurementPointCloud;
   private final Object measurementPointCloudSynchronizer = new Object();

   private List<Point3D32> segmentedPointCloud;
   private double segmentSphereRadius = 1.0;

   private List<Point3D32> localObjectPoints;
   private List<Point3D32> objectInWorldPoints;
   private boolean objectInWorldPointsIsUpToDate = false;

   private final AtomicBoolean useTargetPoint = new AtomicBoolean(true);
   // Point around which ICP will segment the measurement point cloud
   private final FramePoint3D targetPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());

   private final FramePose3D resultPose = new FramePose3D(ReferenceFrame.getWorldFrame());

   public IterativeClosestPointWorker(int numberOfICPObjectPoints, ROS2Helper ros2Helper, Random random)
   {
      this.ros2Helper = ros2Helper;
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;
      this.random = random;

      objectRelativeToCentroidPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      measurementRelativeToCentroidPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      detectionShape = PrimitiveRigidBodyShape.BOX;

      localObjectPoints = IterativeClosestPointTools.createICPObjectPointCloud(detectionShape, new Pose3D(), xLength, yLength, zLength, xRadius, yRadius, zRadius, numberOfICPObjectPoints, random);
      setPoseGuess(resultPose);
   }

   public IterativeClosestPointWorker(PrimitiveRigidBodyShape objectShape,
                                      float xLength,
                                      float yLength,
                                      float zLength,
                                      float xRadius,
                                      float yRadius,
                                      float zRadius,
                                      int numberOfICPObjectPoints,
                                      FramePose3DBasics initialPose,
                                      ROS2Helper ros2Helper,
                                      Random random)
   {
      detectionShape = objectShape;
      this.xLength = xLength;
      this.yLength = yLength;
      this.zLength = zLength;
      this.xRadius = xRadius;
      this.yRadius = yRadius;
      this.zRadius = zRadius;
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;
      this.ros2Helper = ros2Helper;
      this.random = random;

      targetPoint.set(initialPose.getPosition());

      objectRelativeToCentroidPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      measurementRelativeToCentroidPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);

      localObjectPoints = IterativeClosestPointTools.createICPObjectPointCloud(detectionShape, new Pose3D(), xLength, yLength, zLength, xRadius, yRadius, zRadius, numberOfICPObjectPoints, random);
      setPoseGuess(initialPose);
   }

   public boolean runICP(int numberOfIterations)
   {
      // Determine around where the point cloud should be segmented
      // (user provided point or last centroid)
      FramePoint3D detectionPoint;
      if (useTargetPoint.get())
         detectionPoint = new FramePoint3D(targetPoint);
      else
         detectionPoint = new FramePoint3D(resultPose.getPosition());

      // Segment the point cloud
      synchronized (measurementPointCloudSynchronizer) // synchronize as to avoid changes to environment point cloud while segmenting it
      {
         if (measurementPointCloud == null)
            return false;

         segmentPointCloud(measurementPointCloud, detectionPoint, segmentSphereRadius);
      }

      // Only run ICP iteration if segmented point cloud has enough points
      if (segmentedPointCloud.size() >= numberOfICPObjectPoints)
      {
         for (int i = 0; i < numberOfIterations; ++i)
         {
            runICPIteration(segmentedPointCloud);
         }

         return true;
      }

      return false;
   }

   public void publishResults()
   {
      // FIXME this should not be in here.
      // Send result message
      if (sceneNodeID != -1L)
      {
         DetectedObjectPacket resultMessage = new DetectedObjectPacket();
         resultMessage.setId((int) sceneNodeID);
         resultMessage.getPose().set(resultPose);
         for (Point3D32 point3D32 : getObjectPointCloud())
            resultMessage.getObjectPointCloud().add().set(point3D32);
         for (int i = 0; i < numberOfICPObjectPoints; i++)
            resultMessage.getSegmentedPointCloud().add().set(segmentedPointCloud.get(i));
         ros2Helper.publish(PerceptionAPI.ICP_RESULT, resultMessage);
      }
   }

   // TODO: Pass in a Pose to transform (all object points define WRT the pose of object)
   private void runICPIteration(List<Point3D32> segmentedMeasurementPointCloud)
   {
      Collections.shuffle(segmentedMeasurementPointCloud, random);

      // Calculate nearest neighbor for each point (environment to object)
      List<Point3D32> correspondingObjectPoints = new ArrayList<>();
      List<Point3D32> correspondingMeasurementPoints = new ArrayList<>();

      computeCorrespondingPointsBetweenMeasurementAndObjectPointCloud(segmentedMeasurementPointCloud, correspondingMeasurementPoints, correspondingObjectPoints);

      // Calculate object corresponce centroid
      Point3D32 objectCentroid = computeCentroidOfPointCloud(correspondingObjectPoints);

      // TODO I bet there's an element-wise operation for this.
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         objectRelativeToCentroidPoints.set(i, 0, correspondingObjectPoints.get(i).getX() - objectCentroid.getX());
         objectRelativeToCentroidPoints.set(i, 1, correspondingObjectPoints.get(i).getY() - objectCentroid.getY());
         objectRelativeToCentroidPoints.set(i, 2, correspondingObjectPoints.get(i).getZ() - objectCentroid.getZ());
      }

      // Calculate measurement centroid
      Point3D32 measurementCentroid = computeCentroidOfPointCloud(correspondingMeasurementPoints);

      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         measurementRelativeToCentroidPoints.set(i, 0, correspondingMeasurementPoints.get(i).getX() - measurementCentroid.getX());
         measurementRelativeToCentroidPoints.set(i, 1, correspondingMeasurementPoints.get(i).getY() - measurementCentroid.getY());
         measurementRelativeToCentroidPoints.set(i, 2, correspondingMeasurementPoints.get(i).getZ() - measurementCentroid.getZ());
      }

      // Initialize matrix variables
      DMatrixRMaj H = new DMatrixRMaj(3, 3);
      DMatrixRMaj U = new DMatrixRMaj(3, 3);
      DMatrixRMaj V = new DMatrixRMaj(3, 3);
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
      {
         CommonOps_DDRM.scale(-1.0, optimalRotation);
      }

      /* Calculate object transform */
      // Create the transform, and set the rotation, so it's a pure rotation transform
      RigidBodyTransform objectToMeasurementTransform = new RigidBodyTransform();
      objectToMeasurementTransform.getRotation().set(optimalRotation);

      // Calcualte the necessary translation
      objectToMeasurementTransform.transform(objectCentroid, objectAdjustedLocation);
      objectTranslation.sub(measurementCentroid, objectAdjustedLocation);

      // set that translation into the transform
      objectToMeasurementTransform.getTranslation().set(objectTranslation);

      // Rotate and translate object points

      resultPose.applyTransform(objectToMeasurementTransform);
      objectInWorldPointsIsUpToDate = false;

//      updateObjectInWorldPoints();
   }

   private void segmentPointCloud(List<Point3D32> measurementPointCloud, FramePoint3D virtualObjectPointInWorld, double cutoffRange)
   {
      double cutoffSquare = MathTools.square(cutoffRange);

      Stream<Point3D32> measurementStream = useParallelStreams ? measurementPointCloud.parallelStream() : measurementPointCloud.stream();
      segmentedPointCloud = measurementStream.filter(point -> virtualObjectPointInWorld.distanceSquared(point) <= cutoffSquare).collect(Collectors.toList());
      segmentedPointCloud.addAll(measurementPointCloud);
   }

   private void computeCorrespondingPointsBetweenMeasurementAndObjectPointCloud(List<Point3D32> measurementPoints,
                                                                                List<Point3D32> correspondingMeasurementPointsToPack,
                                                                                List<Point3D32> correspondingObjectPointsToPack)
   {
      int measurementIdx = 0;
      int iteration = 0;
      int maxIterations = 2 * numberOfICPObjectPoints;
      List<Point3D32> objectInWorldPoints = getObjectPointCloud();

      while (correspondingMeasurementPointsToPack.size() < numberOfICPObjectPoints && measurementIdx < measurementPoints.size() && iteration < maxIterations)
      {
         Point3D32 measurementPoint = measurementPoints.get(measurementIdx++);
         double minDistance = Double.POSITIVE_INFINITY;
         Point3D32 correspondingObjectPoint = null;

         for (int j = 0; j < objectInWorldPoints.size(); ++j)
         {
            Point3D32 objectPoint = objectInWorldPoints.get(j);
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

   private static Point3D32 computeCentroidOfPointCloud(List<Point3D32> pointCloud)
   {
      return computeCentroidOfPointCloud(pointCloud, pointCloud.size());
   }

   private static Point3D32 computeCentroidOfPointCloud(List<Point3D32> pointCloud, int pointsToAverage)
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
   public void setEnvironmentPointCloud(List<Point3D32> pointCloud)
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

   public void changeSize(float xLength, float yLength, float zLength, float xRadius, float yRadius, float zRadius, int numberOfICPObjectPoints)
   {
      this.xLength = xLength;
      this.yLength = yLength;
      this.zLength = zLength;
      this.xRadius = xRadius;
      this.yRadius = yRadius;
      this.zRadius = zRadius;
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;

      objectRelativeToCentroidPoints.reshape(numberOfICPObjectPoints, 3);
      measurementRelativeToCentroidPoints.reshape(numberOfICPObjectPoints, 3);

      localObjectPoints = IterativeClosestPointTools.createICPObjectPointCloud(detectionShape, new Pose3D(), xLength, yLength, zLength, xRadius, yRadius, zRadius, numberOfICPObjectPoints, random);
      setPoseGuess(resultPose);
   }

   public List<Point3D32> getSegmentedPointCloud()
   {
      return segmentedPointCloud;
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


   public FramePose3DReadOnly getResultPose()
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

   public int getNumberOfICPObjectPoints()
   {
      return numberOfICPObjectPoints;
   }

   public boolean isUsingTargetPoint()
   {
      return useTargetPoint.get();
   }
}
