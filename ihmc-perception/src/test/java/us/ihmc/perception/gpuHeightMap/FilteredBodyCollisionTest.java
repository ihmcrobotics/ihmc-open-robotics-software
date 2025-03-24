package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.tools.PerceptionDebugTools;

import static org.junit.jupiter.api.Assertions.*;

public class FilteredBodyCollisionTest
{
   BodyCollisionFilteredHeightMap bodyCollisionFilteredHeightMap = new BodyCollisionFilteredHeightMap(null);

   @Test
   public void testInSphere()
   {
      Point3D center = new Point3D(0, 0, 0);
      double radius = 5;

      Point3D pointInside = new Point3D(1, 1, 1);
      Point3D pointOutside = new Point3D(10, 10, 10);

      assertTrue(bodyCollisionFilteredHeightMap.isPointInSphere(center, radius, pointInside));
      assertFalse(bodyCollisionFilteredHeightMap.isPointInSphere(center, radius, pointOutside));
   }

   @Test
   public void testFrameCapsule3D()
   {
      Point3D topCenter = new Point3D(0, 0, 5);
      Point3D bottomCenter = new Point3D(0, 0, 0);
      Vector3D axis = new Vector3D(0, 0, 1);
      double radius = 2;

      Point3D pointInsideCapsule = new Point3D(0, 1, 2.5);
      Point3D pointOutsideCapsule = new Point3D(5, 5, 5);
      Point3D pointInsideTopSphere = new Point3D(0, 1, 6);
      Point3D pointInsideBottomSphere = new Point3D(0, 1, -1);

      assertTrue(bodyCollisionFilteredHeightMap.isPointInFrameCapsule3D(topCenter,
                                                                        bottomCenter,
                                                                        axis,
                                                                        radius,
                                                                        pointInsideCapsule));
      assertFalse(bodyCollisionFilteredHeightMap.isPointInFrameCapsule3D(topCenter,
                                                                         bottomCenter,
                                                                         axis,
                                                                         radius,
                                                                         pointOutsideCapsule));
      assertTrue(bodyCollisionFilteredHeightMap.isPointInFrameCapsule3D(topCenter,
                                                                        bottomCenter,
                                                                        axis,
                                                                        radius,
                                                                        pointInsideTopSphere));
      assertTrue(bodyCollisionFilteredHeightMap.isPointInFrameCapsule3D(topCenter,
                                                                        bottomCenter,
                                                                        axis,
                                                                        radius,
                                                                        pointInsideBottomSphere));
   }
}