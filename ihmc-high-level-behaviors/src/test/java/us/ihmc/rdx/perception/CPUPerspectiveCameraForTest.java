package us.ihmc.rdx.perception;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.perception.camera.CameraIntrinsics;

public class CPUPerspectiveCameraForTest
{
   private static final double focalLength = 0.005;

   private final CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();

   private final double imagePlaneWidthM;
   private final double imagePlaneHeightM;

   public CPUPerspectiveCameraForTest(double horizontalFOV, double verticalFOV, int pixelWidth, int pixelHeight)
   {
      cameraIntrinsics.setWidth(pixelWidth);
      cameraIntrinsics.setHeight(pixelHeight);

      double cx = pixelWidth / 2.0;
      double cy = pixelHeight / 2.0;
      cameraIntrinsics.setCx(cx);
      cameraIntrinsics.setCy(cy);

      imagePlaneWidthM = Math.tan(horizontalFOV / 2.0) * focalLength * 2.0;
      imagePlaneHeightM = Math.tan(verticalFOV / 2.0) * focalLength * 2.0;

      // u = x / z * fx + cx -> fx = (u - cx) * z / x. At u = 0, x = -planeWidth / 2.0. At u = pixelWidth, x = width / 2.0
      // v = y / z * fy + cy -> fy = (v - cy) * z / y
      double fx = (pixelWidth - 1 - cx) * focalLength / (imagePlaneWidthM / 2.0);
      double fy = (pixelHeight - 1 - cy) * focalLength / (imagePlaneHeightM / 2.0);
      cameraIntrinsics.setFx(fx);
      cameraIntrinsics.setFy(fy);
   }

   public CameraIntrinsics getCameraIntrinsics()
   {
      return cameraIntrinsics;
   }

   public Vector3DReadOnly computeDirectionOfPixelFromFocalPoint(int u, int v)
   {
      if (u < 0 || u >= cameraIntrinsics.getWidth())
         throw new RuntimeException("u " + u + " is out of bounds");
      if (v < 0 || v >= cameraIntrinsics.getHeight())
         throw new RuntimeException("v " + v + " is out of bounds");

      double planeXDirection = (u - cameraIntrinsics.getCx()) / cameraIntrinsics.getFx();
      double planeYDirection = (v - cameraIntrinsics.getCy()) / cameraIntrinsics.getFy();

      // length is X, Image X is -Y, Image Y is -Z
      Vector3D direction = new Vector3D(1.0, -planeXDirection, -planeYDirection);
      direction.normalize();
      return direction;
   }
}
