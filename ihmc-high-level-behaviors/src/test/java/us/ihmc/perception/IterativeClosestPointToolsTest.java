package us.ihmc.perception;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;

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

         EuclidCoreTestTools.assertEquals("Failed at iter " + iter, correspondanceExpected, correspondance, 1e-6);
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
}
