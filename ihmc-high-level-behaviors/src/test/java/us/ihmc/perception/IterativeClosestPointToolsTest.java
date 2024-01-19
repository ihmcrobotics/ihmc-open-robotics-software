package us.ihmc.perception;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class IterativeClosestPointToolsTest
{
   @Test
   public void testComputeCorrespondencePointOnBox()
   {
      Random random = new Random(1738L);
      int iters = 10000;
      for (int iter = 0; iter < iters; iter++)
      {
         float xLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         float yLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         float zLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         Pose3D boxPose = EuclidGeometryRandomTools.nextPose3D(random, 10.0, 10.0);
         Point3D32 pointToProject = EuclidCoreRandomTools.nextPoint3D32(random);
         pointToProject.scale(20.0);

         Box3D box = new Box3D(boxPose, xLength, yLength, zLength);

         Point3D correspondanceExpected = new Point3D();
         if (!box.isPointInside(pointToProject))
         {
            box.orthogonalProjection(pointToProject, correspondanceExpected);
         }
         else
         {
            // TODO figure this out.
         }

         Point3D32 correspondance = IterativeClosestPointTools.computeCorrespondingPointOnBox(boxPose, pointToProject, xLength, yLength, zLength);

         assertEquals(0.0, box.signedDistance(correspondance), 1e-6);
         assertNotEquals(0.0, box.signedDistance(pointToProject), 1e-6);

//         EuclidCoreTestTools.assertEquals("Failed at iter " + iter, correspondanceExpected, correspondance, 1e-6);
      }

      for (int iter = 0; iter < 100; iter++)
      {
         float xLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         float yLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         float zLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         Pose3D boxPose = EuclidGeometryRandomTools.nextPose3D(random, 10.0, 10.0);
         Box3D box = new Box3D(boxPose, xLength, yLength, zLength);


         List<Point3D32> pointsToProjectA = new ArrayList<>();
         List<Point3D32> pointsToProjectB = new ArrayList<>();
         for (int i = 0; i < 500; i++)
         {
            Point3D32 pointToProject = EuclidCoreRandomTools.nextPoint3D32(random);
            pointToProject.scale(20.0);
            pointsToProjectA.add(pointToProject);
            pointsToProjectB.add(new Point3D32(pointToProject));
         }

         List<Point3DReadOnly> correspondingPoints = new ArrayList<>();
         List<Point3DReadOnly> correspondingPointsOnBox = new ArrayList<>();
         IterativeClosestPointTools.computeCorrespondencesOnShape(PrimitiveRigidBodyShape.BOX,
                                                                  boxPose,
                                                                  pointsToProjectA,
                                                                  correspondingPoints,
                                                                  correspondingPointsOnBox,
                                                                  xLength,
                                                                  yLength,
                                                                  zLength,
                                                                  0.1f,
                                                                  0.1f,
                                                                  0.1f,
                                                                  250);

         // check all the lengths are right.
         assertEquals(250, correspondingPoints.size());
         assertEquals(250, correspondingPointsOnBox.size());
         assertEquals(500, pointsToProjectA.size());

         // check that the input list is unchanged
         for (int i = 0; i < 500; i++)
         {
            EuclidCoreTestTools.assertEquals(pointsToProjectB.get(i), pointsToProjectA.get(i), 1e-6);
         }

         // check all the original points aren't on the box, and all the projected points are on the box
         for (int i = 0; i < 250; i++)
         {
            assertEquals(0.0, box.signedDistance(correspondingPointsOnBox.get(i)), 1e-6);
            assertNotEquals(0.0, box.signedDistance(correspondingPoints.get(i)), 1e-6);
         }

         // check that the centroid of the corresponding points is zero.
         Point3D32 correspondingPointsCentroid = IterativeClosestPointTools.computeCentroidOfPointCloud(correspondingPoints);
         EuclidCoreTestTools.assertEquals(new Point3D(), correspondingPointsCentroid, 1e-4);

         Point3D32 correspondingPointsOnBoxCentroid = IterativeClosestPointTools.computeCentroidOfPointCloud(correspondingPointsOnBox);
         EuclidCoreTestTools.assertEquals(boxPose.getPosition(), correspondingPointsOnBoxCentroid, 1e-4);
      }
   }


   @Test
   public void testProjectingInteriorPointToBoxEdgeRandom()
   {
      Random random = new Random(1738L);
      int boxSizesToCheck = 1000;
      int pointsInEachBoxToCheck = 10000;

      for (int boxIter = 0; boxIter < boxSizesToCheck; boxIter++)
      {
         float xLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         float yLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         float zLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);

         Pose3DReadOnly boxPose = EuclidGeometryRandomTools.nextPose3D(random, 10.0, Math.toRadians(90.0));
         Box3D box = new Box3D(boxPose, xLength, yLength, zLength);

         for (int pointIter = 0; pointIter < pointsInEachBoxToCheck; pointIter++)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 0.5 * xLength, 0.5 * yLength, 0.5 * zLength);
            boxPose.transform(point);

            Point3D32 correspondingPoint = IterativeClosestPointTools.computeCorrespondingPointOnBox(boxPose, point,
                                                                                                     xLength, yLength, zLength);

            // both points should be inside, although the corresponding one maybe not.
            assertTrue(box.isPointInside(point));
            assertTrue(box.isPointInside(correspondingPoint, 1e-3));

            // signed distance should be negative since it's inside, and the orthogonal projection should get the closest point.
            assertEquals(-box.signedDistance(point), correspondingPoint.distance(point), 1e-6);
         }
      }
   }

   @Test
   public void testProjectingInteriorPointToBoxEdge()
   {
      Pose3D boxPose = new Pose3D();

      float xLength = 1.0f;
      float yLength = 0.75f;
      float zLength = 0.5f;

      // test closest to forward x edge
      float xPoint = 0.45f;
      float yPoint = 0.3f;
      float zPoint = 0.15f;
      Point3D32 correspondance = IterativeClosestPointTools.computeCorrespondingPointOnBox(boxPose,
                                                                                           new Point3D(xPoint, yPoint, zPoint),
                                                                                           xLength, yLength, zLength);
      EuclidCoreTestTools.assertEquals(new Point3D(xLength / 2.0f, yPoint, zPoint), correspondance, 1e-5);

      // test closest to backward x edge
      xPoint = -xPoint;
      correspondance = IterativeClosestPointTools.computeCorrespondingPointOnBox(boxPose,
                                                                                 new Point3D(xPoint, yPoint, zPoint),
                                                                                 xLength, yLength, zLength);
      EuclidCoreTestTools.assertEquals(new Point3D(-xLength / 2.0f, yPoint, zPoint), correspondance, 1e-5);

      // test closest to positive y edge
      xPoint = 0.4f;
      yPoint = 0.35f;
      zPoint = 0.15f;
      correspondance = IterativeClosestPointTools.computeCorrespondingPointOnBox(boxPose,
                                                                                           new Point3D(xPoint, yPoint, zPoint),
                                                                                           xLength, yLength, zLength);
      EuclidCoreTestTools.assertEquals(new Point3D(xPoint, yLength / 2.0, zPoint), correspondance, 1e-5);
      // test closest to negative y edge
      xPoint = 0.4f;
      yPoint = -0.35f;
      zPoint = 0.15f;
      correspondance = IterativeClosestPointTools.computeCorrespondingPointOnBox(boxPose,
                                                                                 new Point3D(xPoint, yPoint, zPoint),
                                                                                 xLength, yLength, zLength);
      EuclidCoreTestTools.assertEquals(new Point3D(xPoint, -yLength / 2.0, zPoint), correspondance, 1e-5);

      // test closest to positive z edge
      xPoint = 0.4f;
      yPoint = 0.3f;
      zPoint = 0.22f;
      correspondance = IterativeClosestPointTools.computeCorrespondingPointOnBox(boxPose,
                                                                                 new Point3D(xPoint, yPoint, zPoint),
                                                                                 xLength, yLength, zLength);
      EuclidCoreTestTools.assertEquals(new Point3D(xPoint, yPoint, zLength / 2.0), correspondance, 1e-5);
      // test closest to negative z edge
      xPoint = 0.4f;
      yPoint = 0.3f;
      zPoint = -0.22f;
      correspondance = IterativeClosestPointTools.computeCorrespondingPointOnBox(boxPose,
                                                                                 new Point3D(xPoint, yPoint, zPoint),
                                                                                 xLength, yLength, zLength);
      EuclidCoreTestTools.assertEquals(new Point3D(xPoint, yPoint, -zLength / 2.0), correspondance, 1e-5);


      Random random = new Random(1738L);
      int iters = 10000;
      for (int iter = 0; iter < iters; iter++)
      {
         xLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         yLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);
         zLength = RandomNumbers.nextFloat(random, 0.1f, 5.0f);

         xPoint = RandomNumbers.nextFloat(random, -0.5f, 0.5f) * xLength;
         yPoint = RandomNumbers.nextFloat(random, -0.5f, 0.5f) * yLength;
         zPoint = RandomNumbers.nextFloat(random, -0.5f, 0.5f) * zLength;

         Box3D box = new Box3D(boxPose, xLength, yLength, zLength);


         correspondance = IterativeClosestPointTools.computeCorrespondingPointOnBox(boxPose, new Point3D(xPoint, yPoint, zPoint), xLength, yLength, zLength);

         assertEquals(0.0, box.signedDistance(correspondance), 1e-5);
      }
   }

   @Test
   public void testComputeCentroidOfPointCloud()
   {
      Random random = new Random(1738L);
      int pointsInEachPointCloud = 10000;

      for (int iter = 0; iter < 500; iter++)
      {
         List<Point3DReadOnly> pointCloud = new ArrayList<>();
         Point3D centroid = EuclidCoreRandomTools.nextPoint3D(random, 20.0);

         for (int pointIdx = 0; pointIdx < pointsInEachPointCloud; pointIdx++)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 1.5);
            point.add(centroid);
            pointCloud.add(point);
         }

         Point3D32 computedCentroid = IterativeClosestPointTools.computeCentroidOfPointCloud(pointCloud);
         EuclidCoreTestTools.assertEquals(centroid, computedCentroid, 1e-1);
      }
   }
}
