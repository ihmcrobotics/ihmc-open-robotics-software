package us.ihmc.perception;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import java.util.List;
import java.util.Random;

public class IterativeClosestPointWorker
{
   private final int numberOfICPObjectPoints;

   private final Random random;

   private final OpenCLManager openCLManager;
   private final OpenCLPointCloudExtractor pointCloudExtractor;

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
   private final RecyclingArrayList<Point3D32> objectInWorldPoints;
   private final FramePoint3D detectionPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FramePoint3D lastCentroidPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());

   public IterativeClosestPointWorker(int numberOfICPObjectPoints, OpenCLManager openCLManager, Random random)
   {
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;
      this.openCLManager = openCLManager;
      this.random = random;

      pointCloudExtractor = new OpenCLPointCloudExtractor(openCLManager);

      objectCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentToObjectCorrespondencePoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      objectInWorldPoints = createDefaultBoxPointCloud(numberOfICPObjectPoints, random);
   }

   public IterativeClosestPointWorker(float width, float height, float depth, int numberOfICPObjectPoints, OpenCLManager openCLManager, Random random)
   {
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;
      this.openCLManager = openCLManager;
      this.random = random;

      pointCloudExtractor = new OpenCLPointCloudExtractor(openCLManager);

      objectCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentToObjectCorrespondencePoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      objectInWorldPoints = createICPObjectPointCloudBox(width, height, depth, numberOfICPObjectPoints, random);
   }

   public IterativeClosestPointWorker(PrimitiveRigidBodyShape objectShape,
                                      float width,
                                      float height,
                                      float depth,
                                      float length,
                                      float radius,
                                      int numberOfICPObjectPoints,
                                      OpenCLManager openCLManager,
                                      Random random)
   {
      this.numberOfICPObjectPoints = numberOfICPObjectPoints;
      this.openCLManager = openCLManager;
      this.random = random;

      pointCloudExtractor = new OpenCLPointCloudExtractor(openCLManager);

      objectCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentCentroidSubtractedPoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);
      environmentToObjectCorrespondencePoints = new DMatrixRMaj(numberOfICPObjectPoints, 3);

      switch (objectShape)
      {
         case BOX -> objectInWorldPoints = createICPObjectPointCloudBox(width, height, depth, numberOfICPObjectPoints, random);
         case CONE -> objectInWorldPoints = createICPObjectPointCloudCone(length, radius, numberOfICPObjectPoints, random);
         case CYLINDER -> objectInWorldPoints = createICPObjectPointCloudCylinder(length, radius, numberOfICPObjectPoints, random);
         default -> objectInWorldPoints = createDefaultBoxPointCloud(numberOfICPObjectPoints, random);
      }
   }

   public void setEnvironmentPointCloud(RawImage depthImage)
   {
      depthImage.get();
      synchronized (environmentPointCloudSynchronizer)
      {
         environmentPointCloud = pointCloudExtractor.extractPointCloud(depthImage);
      }
      depthImage.release();
   }

   public void runICP(FixedFramePoint3DBasics targetPoint, double objectCutoffRange, int numberOfIterations)
   {
      if (targetPoint != null)
         detectionPoint.set(targetPoint);
      else
         detectionPoint.set(lastCentroidPoint);

      synchronized (environmentPointCloudSynchronizer)
      {
         if (environmentPointCloud == null)
            return;

         segmentPointCloud(environmentPointCloud, detectionPoint, objectCutoffRange);
      }

      if (segmentedPointCloud.size() >= numberOfICPObjectPoints)
      {
         for (int i = 0; i < numberOfIterations; ++i)
         {
            runICPIteration(segmentedPointCloud);
         }
      }
   }

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

      // Solve for Best Fit Transformation
      CommonOps_DDRM.multTransA(objectCentroidSubtractedPoints, environmentCentroidSubtractedPoints, H);
      svdSolver.decompose(H);
      svdSolver.getU(U, false);
      svdSolver.getV(V, true);
      CommonOps_DDRM.multTransAB(V, U, R);

      // Calculate object translation
      CommonOps_DDRM.multTransB(R, objectCentroid, objectAdjustedLocation);
      CommonOps_DDRM.transpose(objectAdjustedLocation);
      CommonOps_DDRM.subtract(environmentCentroid, objectAdjustedLocation, objectTranslation);

      // Rotate and translate object points
      for (Point3D32 objectInWorldPoint : this.objectInWorldPoints) {
         interimPoint.set(new double[][]{{objectInWorldPoint.getX()}, {objectInWorldPoint.getY()}, {objectInWorldPoint.getZ()}});
         CommonOps_DDRM.mult(R, interimPoint, movedPoint);
         objectInWorldPoint.set(movedPoint.get(0) + objectTranslation.get(0), movedPoint.get(1) + objectTranslation.get(1), movedPoint.get(2) + objectTranslation.get(2));
      }

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
         float x =(float)random.nextDouble(-halfBoxDepth, halfBoxDepth);
         float y =(float)random.nextDouble(-halfBoxWidth, halfBoxWidth);
         float z =(float)random.nextDouble(-halfBoxHeight, halfBoxHeight);
         if (j==0 | j==1) {x = (-(j&1)*halfBoxDepth*2.0f)+halfBoxDepth;}
         if (j==2 | j==3) {y = (-(j&1)*halfBoxWidth*2.0f)+halfBoxWidth;}
         if (j==4 | j==5) {z = (-(j&1)*halfBoxHeight*2.0f)+halfBoxHeight;}
         Point3D32 boxPoint = boxObjectPointCloud.add();
         boxPoint.set(x, y, z);
      }

      return boxObjectPointCloud;
   }

   private RecyclingArrayList<Point3D32> createICPObjectPointCloudCone(float length, float radius, int numberOfPoints, Random random)
   {
      RecyclingArrayList<Point3D32> coneObjectPointCloud = new RecyclingArrayList<>(Point3D32::new);

      for (int i = 0; i < numberOfPoints; i++) {
         float z = (float)random.nextDouble(0, length);
         double phi = random.nextDouble(0, 2*Math.PI);
         float x = (float)Math.cos(phi)*z*(radius/length);
         float y =(float)Math.sin(phi)*z*(radius/length);
         Point3D32 conePoint = coneObjectPointCloud.add();
         conePoint.set(x, y, z);
      }

      return coneObjectPointCloud;
   }

   private RecyclingArrayList<Point3D32> createICPObjectPointCloudCylinder(float length, float radius, int numberOfPoints, Random random)
   {
      RecyclingArrayList<Point3D32> cylinderObjectPointCloud = new RecyclingArrayList<>(Point3D32::new);

      for (int i = 0; i < numberOfPoints; i++) {
         int j = random.nextInt(6);
         float z = (float)random.nextDouble(0, length);
         float r = radius;
         if (j==0) {z = 0; r = (float)random.nextDouble(0, radius);}
         if (j==1) {z = length; r = (float)random.nextDouble(0, radius);}
         double phi = random.nextDouble(0, 2*Math.PI);
         float x = (float)Math.cos(phi)*r;
         float y =(float)Math.sin(phi)*r;
         Point3D32 cylinderPoint = cylinderObjectPointCloud.add();
         cylinderPoint.set(x, y, z);
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
         float x =(float)random.nextDouble(-halfBoxDepth, halfBoxDepth);
         float y =(float)random.nextDouble(-halfBoxWidth, halfBoxWidth);
         float z =(float)random.nextDouble(-halfBoxHeight, halfBoxHeight);
         if (j==0 | j==1) {x = (-(j&1)*halfBoxDepth*2.0f)+halfBoxDepth;}
         if (j==2 | j==3) {y = (-(j&1)*halfBoxWidth*2.0f)+halfBoxWidth;}
         if (j==4 | j==5) {z = (-(j&1)*halfBoxHeight*2.0f)+halfBoxHeight;}
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

   public List<Point3D32> getSegmentedPointCloud()
   {
      return segmentedPointCloud;
   }

   public FixedFramePoint3DBasics getCentroid()
   {
      return lastCentroidPoint;
   }

   public List<Point3D32> getObjectPointcloud()
   {
      return objectInWorldPoints;
   }
}
