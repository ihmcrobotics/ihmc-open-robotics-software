package us.ihmc.perception;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
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
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BinaryOperator;
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

   private boolean useParallelStreams = true;
   private final BinaryOperator<Point3D32> pointSummationOperator = (a, b) ->
   {
      Point3D32 res = new Point3D32(a);
      res.add(b);
      return res;
   };

   private final DMatrixRMaj objectCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj measurementCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj objectPointCloudCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj objectRelativeToCentroidPoints;
   private final DMatrixRMaj measurementRelativeToCentroidPoints;


   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);

   private List<Point3D32> measurementPointCloud;
   private final Object measurementPointCloudSynchronizer = new Object();

   private final RecyclingArrayList<Point3D32> segmentedPointCloud = new RecyclingArrayList<>(Point3D32::new);
   private double segmentSphereRadius = 1.0;

   private List<Point3D32> objectInWorldPoints;

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

      setPoseGuess(initialPose);
   }

   public void runICP(int numberOfIterations)
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
            return;

         segmentPointCloud(measurementPointCloud, detectionPoint, segmentSphereRadius);
      }

      // Only run ICP iteration if segmented point cloud has enough points
      if (segmentedPointCloud.size() >= numberOfICPObjectPoints)
      {
         for (int i = 0; i < numberOfIterations; ++i)
         {
            runICPIteration(segmentedPointCloud);
         }

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
   }

   // TODO: Pass in a Pose to transform (all object points define WRT the pose of object)
   private void runICPIteration(RecyclingArrayList<Point3D32> segmentedMeasurementPointCloud)
   {
      segmentedMeasurementPointCloud.shuffle(random);

      // Calculate nearest neighbor for each point (environment to object)
      List<Point3D32> objectCorrespondingPoints = new ArrayList<>();
      List<Point3D32> measurementPoints = new ArrayList<>();

      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         Point3D32 measurementPoint = segmentedMeasurementPointCloud.get(i);
         double minDistance = Double.POSITIVE_INFINITY;
         int minIndex = -1;

         for (int j = 0; j < objectInWorldPoints.size(); ++j)
         {
            double distanceSquared = measurementPoint.distanceSquared(objectInWorldPoints.get(j));
            if (distanceSquared < minDistance)
            {
               minDistance = distanceSquared;
               minIndex = j;
            }
         }
         if (minIndex == -1)
            return;

          // record
         measurementPoints.add(measurementPoint);
         objectCorrespondingPoints.add(objectInWorldPoints.get(minIndex));
      }

      // Calculate object centroid
      Point3D32 objectCentroid = computeCentroidOfPointCloud(objectCorrespondingPoints);

      // TODO I bet there's an element-wise operation for this.
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         objectRelativeToCentroidPoints.set(i, 0, objectCorrespondingPoints.get(i).getX() - objectCentroid.getX());
         objectRelativeToCentroidPoints.set(i, 1, objectCorrespondingPoints.get(i).getY() - objectCentroid.getY());
         objectRelativeToCentroidPoints.set(i, 2, objectCorrespondingPoints.get(i).getZ() - objectCentroid.getZ());
      }

      // Calculate measurement centroid
      Point3D32 measurementCentroid = computeCentroidOfPointCloud(measurementPoints);

      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         measurementRelativeToCentroidPoints.set(i, 0, segmentedMeasurementPointCloud.get(i).getX() - measurementCentroid.getX());
         measurementRelativeToCentroidPoints.set(i, 1, segmentedMeasurementPointCloud.get(i).getY() - measurementCentroid.getY());
         measurementRelativeToCentroidPoints.set(i, 2, segmentedMeasurementPointCloud.get(i).getZ() - measurementCentroid.getZ());
      }

      // Initialize matrix variables
      DMatrixRMaj H = new DMatrixRMaj(3, 3);
      DMatrixRMaj U = new DMatrixRMaj(3, 3);
      DMatrixRMaj V = new DMatrixRMaj(3, 3);
      DMatrixRMaj R = new DMatrixRMaj(3, 3); // This is the optimized rotation matrix to rotate the measurement to match the point cloud.
      DMatrixRMaj objectAdjustedLocation = new DMatrixRMaj(3, 1);
      DMatrixRMaj objectTranslation = new DMatrixRMaj(1, 3);
      // TODO this isn't anything
      DMatrixRMaj T = new DMatrixRMaj(4, 4);
      CommonOps_DDRM.setIdentity(T);

      // TODO: Get new translation every iteration, append to last, modify object pose using the translation
      // Solve for Best Fit Transformation
      CommonOps_DDRM.multTransA(objectRelativeToCentroidPoints, measurementRelativeToCentroidPoints, H);
      svdSolver.decompose(H);
      svdSolver.getU(U, false);
      svdSolver.getV(V, true);
      CommonOps_DDRM.multTransAB(V, U, R);

      // Correct any negative rotation matrices which come out of SVD its positive counterpart (fixes NotARotationMatrix issues)
      if (CommonOps_DDRM.det(R) < 0.0)
      {
         CommonOps_DDRM.scale(-1.0, R);
      }

      /* Calculate object transform */
      // Create the transform, and set the rotation, so it's a pure rotation transform
      this.objectCentroid.set(0, objectCentroid.getX());
      this.objectCentroid.set(1, objectCentroid.getY());
      this.objectCentroid.set(2, objectCentroid.getZ());
      this.measurementCentroid.set(0, measurementCentroid.getX());
      this.measurementCentroid.set(1, measurementCentroid.getY());
      this.measurementCentroid.set(2, measurementCentroid.getZ());
      CommonOps_DDRM.multTransB(R, this.objectCentroid, objectAdjustedLocation);
      CommonOps_DDRM.transpose(objectAdjustedLocation);
      CommonOps_DDRM.subtract(this.measurementCentroid, objectAdjustedLocation, objectTranslation);

      RotationMatrix deltaRotation = new RotationMatrix(R);
      RigidBodyTransform worldToPointsTransform = new RigidBodyTransform();
      worldToPointsTransform.getRotation().set(R);
      worldToPointsTransform.getTranslation().set(objectTranslation.get(0), objectTranslation.get(1), objectTranslation.get(2));

      // Rotate and translate object points
      // TODO why is this all the points? We only need to transform the points that will be used in to compute the centroid.
      Stream<Point3D32> pointsStream = useParallelStreams ? objectInWorldPoints.parallelStream() : objectInWorldPoints.stream();
      pointsStream.forEach(objectInWorldPoint -> objectInWorldPoint.applyTransform(worldToPointsTransform));

      // TODO: Remove this cheat
      // Calculate object centroid from the corrected points
      Point3D32 objectPointCloudCentroid = computeCentroidOfPointCloud(objectInWorldPoints, numberOfICPObjectPoints);

      resultPose.prependRotation(deltaRotation);
      resultPose.getPosition().set(objectPointCloudCentroid);
   }

   private void segmentPointCloud(List<Point3D32> measurementPointCloud, FramePoint3D virtualObjectPointInWorld, double cutoffRange)
   {
      // TODO use array streams and filtered points.
      segmentedPointCloud.clear();

      double cutoffSquare = MathTools.square(cutoffRange);
      for (Point3D32 point : measurementPointCloud)
      {
         double distanceFromVirtualObjectCentroid = virtualObjectPointInWorld.distanceSquared(point);
         if (distanceFromVirtualObjectCentroid <= cutoffSquare)
         {
            Point3D32 segmentPoint = segmentedPointCloud.add();
            segmentPoint.set(point);
         }
         else
         {
            LogTools.info("Crap");
         }
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

      setPoseGuess(resultPose);
   }

   public List<Point3D32> getSegmentedPointCloud()
   {
      return segmentedPointCloud;
   }

   public void setPoseGuess(Pose3DReadOnly pose)
   {
      resultPose.set(pose);
      objectInWorldPoints = IterativeClosestPointTools.createICPObjectPointCloud(detectionShape, resultPose, xLength, yLength, zLength, xRadius, yRadius, zRadius, numberOfICPObjectPoints, random);
   }

   public FramePose3DReadOnly getResultPose()
   {
      return resultPose;
   }

   public List<Point3D32> getObjectPointCloud()
   {
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
