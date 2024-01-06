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
}
