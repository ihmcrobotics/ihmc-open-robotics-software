package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.tools.PerceptionDebugTools;

public class FilteredBodyCollisionTest
{
   BodyCollisionFilteredHeightMap bodyCollisionFilteredHeightMap = new BodyCollisionFilteredHeightMap();

   @Test
   public void testInSphere()
   {
      Point3D center = new Point3D(0, 0, 0);  // Center of the sphere
      double radius = 5;  // Radius of the sphere
      Point3D point = new Point3D(3, 4, 0);  // Point to check

      if (bodyCollisionFilteredHeightMap.isPointInSphere(center, radius, point))
      {
         System.out.println("The point is inside the sphere.");
      }
      else
      {
         System.out.println("The point is outside the sphere.");
      }
   }
}

