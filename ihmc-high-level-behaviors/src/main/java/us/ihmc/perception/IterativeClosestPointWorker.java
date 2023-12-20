package us.ihmc.perception;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import perception_msgs.msg.dds.DetectedObjectPacket;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

public class IterativeClosestPointWorker
{
   private final Random random;

   private final ROS2Helper ros2Helper;
   private long sceneNodeID = -1L;

   private final PrimitiveRigidBodyShape detectionShape;
   private float width = 0.0f;
   private float height = 0.0f;
   private float depth = 0.0f;
   private float length = 0.0f;
   private float radius = 0.0f;
   private int numberOfICPObjectPoints;

   private final DMatrixRMaj objectCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj environmentCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj objectPointCloudCentroid = new DMatrixRMaj(1, 3);
   private final DMatrixRMaj objectCentroidSubtractedPoints;
   private final DMatrixRMaj environmentCentroidSubtractedPoints;
   private final DMatrixRMaj environmentToObjectCorrespondencePoints;

   private final SvdImplicitQrDecompose_DDRM svdSolver = new SvdImplicitQrDecompose_DDRM(false, true, true, false);

   private RecyclingArrayList<Point3D32> environmentPointCloud;
   private final Object environmentPointCloudSynchronizer = new Object();

   private final RecyclingArrayList<Point3D32> segmentedPointCloud = new RecyclingArrayList<>(Point3D32::new);
   private double segmentSphereRadius = 0.3;

   private RecyclingArrayList<Point3D32> objectInWorldPoints;

   private final AtomicBoolean useTargetPoint = new AtomicBoolean(true);
   // Point around which ICP will segment the environment point cloud
   private final FramePoint3D targetPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
   // This is the centroid of the ICP object point cloud, updated after each iteration
   private final FramePoint3D lastCentroidPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
   // FIXME: Calculations of this orientation are incorrect
   private final Quaternion orientation = new Quaternion();

   public IterativeClosestPointWorker(int numberOfICPObjectPoints, ROS2Helper ros2Helper, Random random)
   {
      this.ros2Helper = ros2Helper;
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;
      this.random = random;

      objectCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentToObjectCorrespondencePoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      detectionShape = PrimitiveRigidBodyShape.BOX;
      objectInWorldPoints = createDefaultBoxPointCloud(numberOfICPObjectPoints, random);
   }

   public IterativeClosestPointWorker(PrimitiveRigidBodyShape objectShape,
                                      float width,
                                      float height,
                                      float depth,
                                      float length,
                                      float radius,
                                      int numberOfICPObjectPoints,
                                      ROS2Helper ros2Helper,
                                      Random random)
   {
      this.ros2Helper = ros2Helper;
      this.width = width;
      this.height = height;
      this.depth = depth;
      this.length = length;
      this.radius = radius;
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;
      this.random = random;
      detectionShape = objectShape;

      objectCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentToObjectCorrespondencePoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);

      switch (detectionShape)
      {
         case BOX -> objectInWorldPoints = createICPObjectPointCloudBox(width, height, depth, numberOfICPObjectPoints, random);
         case CONE -> objectInWorldPoints = createICPObjectPointCloudCone(length, radius, numberOfICPObjectPoints, random);
         case CYLINDER -> objectInWorldPoints = createICPObjectPointCloudCylinder(length, radius, numberOfICPObjectPoints, random);
         default -> objectInWorldPoints = createDefaultBoxPointCloud(numberOfICPObjectPoints, random);
      }
   }

   public void runICP(int numberOfIterations)
   {
      // Determine around where the point cloud should be segmented
      // (user provided point or last centroid)
      FramePoint3D detectionPoint;
      if (useTargetPoint.get())
         detectionPoint = new FramePoint3D(targetPoint);
      else
         detectionPoint = new FramePoint3D(lastCentroidPoint);

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
      }

      // Send result message
      if (sceneNodeID != -1L)
      {
         DetectedObjectPacket resultMessage = new DetectedObjectPacket();
         resultMessage.setId((int) sceneNodeID);
         resultMessage.getPose().set(orientation, lastCentroidPoint);
         ros2Helper.publish(PerceptionAPI.ICP_RESULT, resultMessage);
      }
   }

   // TODO: Pass in a Pose to transform (all object points define WRT the pose of object)
   private void runICPIteration(RecyclingArrayList<Point3D32> segmentedEnvironmentPointCloud)
   {
      segmentedEnvironmentPointCloud.shuffle(random);

      // Calculate nearest neighbor (environment to object)
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         Point3D32 environmentPoint = segmentedEnvironmentPointCloud.get(i);
         float minDistance = Float.MAX_VALUE;
         int minIndex = Integer.MAX_VALUE;

         for (int j = 0; j < objectInWorldPoints.size(); ++j)
         {
            float distance = (float) environmentPoint.distance(objectInWorldPoints.get(j));
            if (distance <= minDistance)
            {
               minDistance = distance;
               minIndex = j;
            }
         }
         if (minIndex == Integer.MAX_VALUE)
            return;

         environmentToObjectCorrespondencePoints.set(i, 0, objectInWorldPoints.get(minIndex).getX());
         environmentToObjectCorrespondencePoints.set(i, 1, objectInWorldPoints.get(minIndex).getY());
         environmentToObjectCorrespondencePoints.set(i, 2, objectInWorldPoints.get(minIndex).getZ());
      }

      // Calculate object centroid
      objectCentroid.set(0, 0, 0);
      objectCentroid.set(0, 1, 0);
      objectCentroid.set(0, 2, 0);
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         objectCentroid.add(0, 0, environmentToObjectCorrespondencePoints.get(i, 0));
         objectCentroid.add(0, 1, environmentToObjectCorrespondencePoints.get(i, 1));
         objectCentroid.add(0, 2, environmentToObjectCorrespondencePoints.get(i, 2));
      }
      objectCentroid.set(0, 0, objectCentroid.get(0, 0) / numberOfICPObjectPoints);
      objectCentroid.set(0, 1, objectCentroid.get(0, 1) / numberOfICPObjectPoints);
      objectCentroid.set(0, 2, objectCentroid.get(0, 2) / numberOfICPObjectPoints);

      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         objectCentroidSubtractedPoints.set(i, 0, environmentToObjectCorrespondencePoints.get(i, 0) - objectCentroid.get(0, 0));
         objectCentroidSubtractedPoints.set(i, 1, environmentToObjectCorrespondencePoints.get(i, 1) - objectCentroid.get(0, 1));
         objectCentroidSubtractedPoints.set(i, 2, environmentToObjectCorrespondencePoints.get(i, 2) - objectCentroid.get(0, 2));
      }

      // Calculate environment centroid
      environmentCentroid.set(0, 0, 0);
      environmentCentroid.set(0, 1, 0);
      environmentCentroid.set(0, 2, 0);
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         environmentCentroid.add(0, 0, segmentedEnvironmentPointCloud.get(i).getX());
         environmentCentroid.add(0, 1, segmentedEnvironmentPointCloud.get(i).getY());
         environmentCentroid.add(0, 2, segmentedEnvironmentPointCloud.get(i).getZ());
      }
      environmentCentroid.set(0, 0, environmentCentroid.get(0, 0) / numberOfICPObjectPoints);
      environmentCentroid.set(0, 1, environmentCentroid.get(0, 1) / numberOfICPObjectPoints);
      environmentCentroid.set(0, 2, environmentCentroid.get(0, 2) / numberOfICPObjectPoints);

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
      DMatrixRMaj R = new DMatrixRMaj(3, 3);
      DMatrixRMaj objectAdjustedLocation = new DMatrixRMaj(3, 1);
      DMatrixRMaj objectTranslation = new DMatrixRMaj(1, 3);
      DMatrixRMaj T = new DMatrixRMaj(4, 4);
      CommonOps_DDRM.setIdentity(T);
      DMatrixRMaj interimPoint = new DMatrixRMaj(1, 3);
      DMatrixRMaj movedPoint = new DMatrixRMaj(1, 3);

      // TODO: Get new translation every iteration, append to last, modify object pose using the translation
      // Solve for Best Fit Transformation
      CommonOps_DDRM.multTransA(objectCentroidSubtractedPoints, environmentCentroidSubtractedPoints, H);
      svdSolver.decompose(H);
      svdSolver.getU(U, false);
      svdSolver.getV(V, true);
      CommonOps_DDRM.multTransAB(V, U, R);

      Quaternion quaternionRotatioin = new Quaternion();
      quaternionRotatioin.setRotationMatrix(R.get(0, 0),
                                            R.get(0, 1),
                                            R.get(0, 2),
                                            R.get(1, 0),
                                            R.get(1, 1),
                                            R.get(1, 2),
                                            R.get(2, 0),
                                            R.get(2, 1),
                                            R.get(2, 2));
      orientation.append(quaternionRotatioin);

      // Calculate object translation
      CommonOps_DDRM.multTransB(R, objectCentroid, objectAdjustedLocation);
      CommonOps_DDRM.transpose(objectAdjustedLocation);
      CommonOps_DDRM.subtract(environmentCentroid, objectAdjustedLocation, objectTranslation);

      // Rotate and translate object points
      for (Point3D32 objectInWorldPoint : this.objectInWorldPoints)
      {
         interimPoint.set(new double[][] {{objectInWorldPoint.getX()}, {objectInWorldPoint.getY()}, {objectInWorldPoint.getZ()}});
         CommonOps_DDRM.mult(R, interimPoint, movedPoint);
         objectInWorldPoint.set(movedPoint.get(0) + objectTranslation.get(0),
                                movedPoint.get(1) + objectTranslation.get(1),
                                movedPoint.get(2) + objectTranslation.get(2));
      }

      // TODO: Remove this cheat
      // Calculate object centroid
      objectPointCloudCentroid.set(0, 0, 0);
      objectPointCloudCentroid.set(0, 1, 0);
      objectPointCloudCentroid.set(0, 2, 0);
      for (int i = 0; i < numberOfICPObjectPoints; ++i)
      {
         objectPointCloudCentroid.add(0, 0, objectInWorldPoints.get(i).getX());
         objectPointCloudCentroid.add(0, 1, objectInWorldPoints.get(i).getY());
         objectPointCloudCentroid.add(0, 2, objectInWorldPoints.get(i).getZ());
      }
      objectPointCloudCentroid.set(0, 0, objectPointCloudCentroid.get(0, 0) / numberOfICPObjectPoints);
      objectPointCloudCentroid.set(0, 1, objectPointCloudCentroid.get(0, 1) / numberOfICPObjectPoints);
      objectPointCloudCentroid.set(0, 2, objectPointCloudCentroid.get(0, 2) / numberOfICPObjectPoints);
      lastCentroidPoint.set(objectPointCloudCentroid.get(0, 0), objectPointCloudCentroid.get(0, 1), objectPointCloudCentroid.get(0, 2));
   }

   private RecyclingArrayList<Point3D32> createICPObjectPointCloudBox(float width, float height, float depth, int numberOfPoints, Random random)
   {
      RecyclingArrayList<Point3D32> boxObjectPointCloud = new RecyclingArrayList<>(Point3D32::new);

      float halfBoxWidth = width / 2.0f;
      float halfBoxDepth = height / 2.0f;
      float halfBoxHeight = depth / 2.0f;
      for (int i = 0; i < numberOfPoints; i++)
      {
         int j = random.nextInt(6);
         float x = (float) random.nextDouble(-halfBoxDepth, halfBoxDepth);
         float y = (float) random.nextDouble(-halfBoxWidth, halfBoxWidth);
         float z = (float) random.nextDouble(-halfBoxHeight, halfBoxHeight);
         if (j == 0 | j == 1)
         {
            x = (-(j & 1) * halfBoxDepth * 2.0f) + halfBoxDepth;
         }
         if (j == 2 | j == 3)
         {
            y = (-(j & 1) * halfBoxWidth * 2.0f) + halfBoxWidth;
         }
         if (j == 4 | j == 5)
         {
            z = (-(j & 1) * halfBoxHeight * 2.0f) + halfBoxHeight;
         }
         Point3D32 boxPoint = boxObjectPointCloud.add();
         boxPoint.set(lastCentroidPoint);
         boxPoint.add(x, y, z);
      }

      return boxObjectPointCloud;
   }

   private RecyclingArrayList<Point3D32> createICPObjectPointCloudCone(float length, float radius, int numberOfPoints, Random random)
   {
      RecyclingArrayList<Point3D32> coneObjectPointCloud = new RecyclingArrayList<>(Point3D32::new);

      for (int i = 0; i < numberOfPoints; i++)
      {
         float z = (float) random.nextDouble(0, length);
         double phi = random.nextDouble(0, 2 * Math.PI);
         float x = (float) Math.cos(phi) * z * (radius / length);
         float y = (float) Math.sin(phi) * z * (radius / length);
         Point3D32 conePoint = coneObjectPointCloud.add();
         conePoint.set(lastCentroidPoint);
         conePoint.add(x, y, z);
      }

      return coneObjectPointCloud;
   }

   private RecyclingArrayList<Point3D32> createICPObjectPointCloudCylinder(float length, float radius, int numberOfPoints, Random random)
   {
      RecyclingArrayList<Point3D32> cylinderObjectPointCloud = new RecyclingArrayList<>(Point3D32::new);

      for (int i = 0; i < numberOfPoints; i++)
      {
         int j = random.nextInt(6);
         float z = (float) random.nextDouble(0, length);
         float r = radius;
         if (j == 0)
         {
            z = 0;
            r = (float) random.nextDouble(0, radius);
         }
         if (j == 1)
         {
            z = length;
            r = (float) random.nextDouble(0, radius);
         }
         double phi = random.nextDouble(0, 2 * Math.PI);
         float x = (float) Math.cos(phi) * r;
         float y = (float) Math.sin(phi) * r;
         Point3D32 cylinderPoint = cylinderObjectPointCloud.add();
         cylinderPoint.set(lastCentroidPoint);
         cylinderPoint.add(x, y, z);
      }

      return cylinderObjectPointCloud;
   }

   private RecyclingArrayList<Point3D32> createDefaultBoxPointCloud(int numberOfPoints, Random random)
   {
      RecyclingArrayList<Point3D32> boxObjectPointCloud = new RecyclingArrayList<>(Point3D32::new);

      float halfBoxWidth = 0.405f / 2.0f;
      float halfBoxDepth = 0.31f / 2.0f;
      float halfBoxHeight = 0.19f / 2.0f;
      for (int i = 0; i < numberOfPoints; i++)
      {
         int j = random.nextInt(6);
         float x = (float) random.nextDouble(-halfBoxDepth, halfBoxDepth);
         float y = (float) random.nextDouble(-halfBoxWidth, halfBoxWidth);
         float z = (float) random.nextDouble(-halfBoxHeight, halfBoxHeight);
         if (j == 0 | j == 1)
         {
            x = (-(j & 1) * halfBoxDepth * 2.0f) + halfBoxDepth;
         }
         if (j == 2 | j == 3)
         {
            y = (-(j & 1) * halfBoxWidth * 2.0f) + halfBoxWidth;
         }
         if (j == 4 | j == 5)
         {
            z = (-(j & 1) * halfBoxHeight * 2.0f) + halfBoxHeight;
         }
         Point3D32 boxPoint = boxObjectPointCloud.add();
         boxPoint.set(x, y, z);
      }

      return boxObjectPointCloud;
   }

   private void segmentPointCloud(List<Point3D32> environmentPointCloud, FramePoint3D virtualObjectPointInWorld, double cutoffRange)
   {
      segmentedPointCloud.clear();

      for (Point3D32 point : environmentPointCloud)
      {
         double distanceFromVirtualObjectCentroid = virtualObjectPointInWorld.distance(point);
         if (distanceFromVirtualObjectCentroid <= cutoffRange)
         {
            Point3D32 segmentPoint = segmentedPointCloud.add();
            segmentPoint.set(point);
         }
      }
   }

   // TODO: Color filtering could go here
   // pass in depth and color image -> cut out invalid pixels of depth image based on color image ->
   // -> create "environmentPointCloud" based off the color-filtered depth image ->
   // -> set the segmentation radius to be large (1.0~1.5?)
   public void setEnvironmentPointCloud(RecyclingArrayList<Point3D32> pointCloud)
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

   public void changeSize(float width, float height, float depth, float length, float radius, int numberOfICPObjectPoints)
   {
      this.width = width;
      this.height = height;
      this.depth = depth;
      this.length = length;
      this.radius = radius;
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;
      switch (detectionShape)
      {
         case BOX -> objectInWorldPoints = createICPObjectPointCloudBox(width, height, depth, numberOfICPObjectPoints, random);
         case CYLINDER -> objectInWorldPoints = createICPObjectPointCloudCylinder(length, radius, numberOfICPObjectPoints, random);
         case CONE -> objectInWorldPoints = createICPObjectPointCloudCone(length, radius, numberOfICPObjectPoints, random);
      }
   }

   public List<Point3D32> getSegmentedPointCloud()
   {
      return segmentedPointCloud;
   }

   public FixedFramePoint3DBasics getCentroid()
   {
      return lastCentroidPoint;
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   public List<Point3D32> getObjectPointCloud()
   {
      return objectInWorldPoints;
   }

   public float getWidth()
   {
      return width;
   }

   public float getHeight()
   {
      return height;
   }

   public float getDepth()
   {
      return depth;
   }

   public float getLength()
   {
      return length;
   }

   public float getRadius()
   {
      return radius;
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
