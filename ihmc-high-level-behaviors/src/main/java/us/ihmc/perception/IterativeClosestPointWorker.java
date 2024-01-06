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
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.tools.lists.PairList;

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
   private final DMatrixRMaj environmentCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj objectPointCloudCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj objectCentroidSubtractedPoints;
   private final DMatrixRMaj environmentCentroidSubtractedPoints;
   private final DMatrixRMaj environmentToObjectCorrespondencePoints;


   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);

   private List<Point3D32> environmentPointCloud;
   private final Object environmentPointCloudSynchronizer = new Object();

   private final RecyclingArrayList<Point3D32> segmentedPointCloud = new RecyclingArrayList<>(Point3D32::new);
   private double segmentSphereRadius = 1.0;

   private List<Point3D32> objectInWorldPoints;

   private final AtomicBoolean useTargetPoint = new AtomicBoolean(true);
   // Point around which ICP will segment the environment point cloud
   private final FramePoint3D targetPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());

   private final FramePose3D resultPose = new FramePose3D(ReferenceFrame.getWorldFrame());

   public IterativeClosestPointWorker(int numberOfICPObjectPoints, ROS2Helper ros2Helper, Random random)
   {
      this.ros2Helper = ros2Helper;
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;
      this.random = random;

      objectCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentToObjectCorrespondencePoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
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

      objectCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentToObjectCorrespondencePoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);

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
      synchronized (environmentPointCloudSynchronizer) // synchronize as to avoid changes to environment point cloud while segmenting it
      {
         if (environmentPointCloud == null)
            return;

         segmentPointCloud(environmentPointCloud, detectionPoint, segmentSphereRadius);
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
   private void runICPIteration(RecyclingArrayList<Point3D32> segmentedEnvironmentPointCloud)
   {
      segmentedEnvironmentPointCloud.shuffle(random);

      // Calculate nearest neighbor for each point (environment to object)
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         Point3D32 environmentPoint = segmentedEnvironmentPointCloud.get(i);
         double minDistance = Double.POSITIVE_INFINITY;
         int minIndex = -1;

         for (int j = 0; j < objectInWorldPoints.size(); ++j)
         {
            double distanceSquared = environmentPoint.distanceSquared(objectInWorldPoints.get(j));
            if (distanceSquared < minDistance)
            {
               minDistance = distanceSquared;
               minIndex = j;
            }
         }
         if (minIndex == -1)
            return;

          // record
         environmentToObjectCorrespondencePoints.set(i, 0, objectInWorldPoints.get(minIndex).getX());
         environmentToObjectCorrespondencePoints.set(i, 1, objectInWorldPoints.get(minIndex).getY());
         environmentToObjectCorrespondencePoints.set(i, 2, objectInWorldPoints.get(minIndex).getZ());
      }

      // Calculate object centroid
      objectCentroid.zero();
      CommonOps_DDRM.sumCols(environmentToObjectCorrespondencePoints, objectCentroid);
      CommonOps_DDRM.scale(1.0 / numberOfICPObjectPoints, objectCentroid);

      // TODO I bet there's an element-wise operation for this.
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         objectCentroidSubtractedPoints.set(i, 0, environmentToObjectCorrespondencePoints.get(i, 0) - objectCentroid.get(0, 0));
         objectCentroidSubtractedPoints.set(i, 1, environmentToObjectCorrespondencePoints.get(i, 1) - objectCentroid.get(0, 1));
         objectCentroidSubtractedPoints.set(i, 2, environmentToObjectCorrespondencePoints.get(i, 2) - objectCentroid.get(0, 2));
      }

      // Calculate environment centroid
      environmentCentroid.zero();
      // FIXME this is a super gross way of doing this, but may be unavoidable.
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         environmentCentroid.add(0, 0, segmentedEnvironmentPointCloud.get(i).getX());
         environmentCentroid.add(0, 1, segmentedEnvironmentPointCloud.get(i).getY());
         environmentCentroid.add(0, 2, segmentedEnvironmentPointCloud.get(i).getZ());
      }
      CommonOps_DDRM.scale(1.0 / numberOfICPObjectPoints, environmentCentroid);

      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         environmentCentroidSubtractedPoints.set(i, 0, segmentedEnvironmentPointCloud.get(i).getX() - environmentCentroid.get(0, 0));
         environmentCentroidSubtractedPoints.set(i, 1, segmentedEnvironmentPointCloud.get(i).getY() - environmentCentroid.get(0, 1));
         environmentCentroidSubtractedPoints.set(i, 2, segmentedEnvironmentPointCloud.get(i).getZ() - environmentCentroid.get(0, 2));
      }

      // Initialize matrix variables
      DMatrixRMaj H = new DMatrixRMaj(3, 3);
      DMatrixRMaj U = new DMatrixRMaj(3, 3);
      DMatrixRMaj V = new DMatrixRMaj(3, 3);
      DMatrixRMaj R = new DMatrixRMaj(3, 3); // This is the optimized rotation matrix to rotate the environment to match the point cloud.
      DMatrixRMaj objectAdjustedLocation = new DMatrixRMaj(3, 1);
      DMatrixRMaj objectTranslation = new DMatrixRMaj(1, 3);
      // TODO this isn't anything
      DMatrixRMaj T = new DMatrixRMaj(4, 4);
      CommonOps_DDRM.setIdentity(T);

      // TODO: Get new translation every iteration, append to last, modify object pose using the translation
      // Solve for Best Fit Transformation
      CommonOps_DDRM.multTransA(objectCentroidSubtractedPoints, environmentCentroidSubtractedPoints, H);
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
      CommonOps_DDRM.multTransB(R, objectCentroid, objectAdjustedLocation);
      CommonOps_DDRM.transpose(objectAdjustedLocation);
      CommonOps_DDRM.subtract(environmentCentroid, objectAdjustedLocation, objectTranslation);

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
      objectPointCloudCentroid.zero();
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         objectPointCloudCentroid.add(0, 0, objectInWorldPoints.get(i).getX());
         objectPointCloudCentroid.add(0, 1, objectInWorldPoints.get(i).getY());
         objectPointCloudCentroid.add(0, 2, objectInWorldPoints.get(i).getZ());
      }
      CommonOps_DDRM.scale(1.0 / numberOfICPObjectPoints, objectPointCloudCentroid);

      resultPose.prependRotation(deltaRotation);
      resultPose.getPosition().set(objectPointCloudCentroid.get(0, 0), objectPointCloudCentroid.get(0, 1), objectPointCloudCentroid.get(0, 2));
   }

   private void segmentPointCloud(List<Point3D32> environmentPointCloud, FramePoint3D virtualObjectPointInWorld, double cutoffRange)
   {
      // TODO use array streams and filtered points.
      segmentedPointCloud.clear();

      double cutoffSquare = MathTools.square(cutoffRange);
      for (Point3D32 point : environmentPointCloud)
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
      Point3D32 centroid = new Point3D32();
      for (int i = 0; i < pointCloud.size(); i++)
         centroid.add(pointCloud.get(i));
      centroid.scale(1.0 / pointCloud.size());

      return centroid;
   }


   // TODO: Color filtering could go here
   // pass in depth and color image -> cut out invalid pixels of depth image based on color image ->
   // -> create "environmentPointCloud" based off the color-filtered depth image ->
   // -> set the segmentation radius to be large (1.0~1.5?)
   public void setEnvironmentPointCloud(List<Point3D32> pointCloud)
   {
      synchronized (environmentPointCloudSynchronizer)
      {
         environmentPointCloud = pointCloud;
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

      objectCentroidSubtractedPoints.reshape(numberOfICPObjectPoints, 3);
      environmentCentroidSubtractedPoints.reshape(numberOfICPObjectPoints, 3);
      environmentToObjectCorrespondencePoints.reshape(numberOfICPObjectPoints, 3);

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
