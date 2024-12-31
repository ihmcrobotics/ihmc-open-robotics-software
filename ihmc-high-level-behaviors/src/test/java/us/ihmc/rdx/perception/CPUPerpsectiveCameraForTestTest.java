package us.ihmc.rdx.perception;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class CPUPerpsectiveCameraForTestTest
{
   @Test
   public void testIntrinsicsCalculation()
   {
      double horizontalFOV = Math.toRadians(90);
      double verticalFOV = Math.toRadians(70);
      CPUPerspectiveCameraForTest camera = new CPUPerspectiveCameraForTest(horizontalFOV, verticalFOV, 1024, 720);

      // Test in the middle
      int middleU = (int) camera.getCameraIntrinsics().getCx();
      int middleV = (int) camera.getCameraIntrinsics().getCy();
      Vector3DReadOnly directionAtCenter = camera.computeDirectionOfPixelFromFocalPoint(middleU, middleV);
      Vector3D expectedCenter = new Vector3D(1.0, 0, 0);

      EuclidCoreTestTools.assertEquals(expectedCenter, directionAtCenter, 1e-3);

      // Test at the far left
      Vector3DReadOnly directionAtFarLeft = camera.computeDirectionOfPixelFromFocalPoint(0, middleV);
      Vector3D expectedDirectionAtFarLeft = new Vector3D(Math.cos(horizontalFOV / 2.0), Math.sin(horizontalFOV / 2.0), 0.0);
      EuclidCoreTestTools.assertEquals(expectedDirectionAtFarLeft, directionAtFarLeft, 1e-3);

      // Test at the far right
      Vector3DReadOnly directionAtFarRight = camera.computeDirectionOfPixelFromFocalPoint(1023, middleV);
      Vector3D expectedDirectionAtFarRight = new Vector3D(Math.cos(horizontalFOV / 2.0), -Math.sin(horizontalFOV / 2.0), 0.0);
      EuclidCoreTestTools.assertEquals(expectedDirectionAtFarRight, directionAtFarRight, 1e-3);

      // Test at the top
      Vector3DReadOnly directionATop = camera.computeDirectionOfPixelFromFocalPoint(middleU, 0);
      Vector3D expectedDirectionTop = new Vector3D(Math.cos(verticalFOV / 2.0), 0.0, Math.sin(verticalFOV / 2.0));
      EuclidCoreTestTools.assertEquals(expectedDirectionTop, directionATop, 2e-3);

      // Test at the far right
      Vector3DReadOnly directionAtBottom = camera.computeDirectionOfPixelFromFocalPoint(middleU, 719);
      Vector3D expectedDirectionBottom = new Vector3D(Math.cos(verticalFOV / 2.0), 0.0, -Math.sin(verticalFOV / 2.0));
      EuclidCoreTestTools.assertEquals(expectedDirectionBottom, directionAtBottom, 1e-3);
   }
}
