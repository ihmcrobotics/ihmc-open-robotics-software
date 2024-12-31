package us.ihmc.rdx.perception.heightMap;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.robotics.referenceFrames.ZUpFrame;

public class CPUPerspectiveCameraForTest
{
   private static final double focalLength = 0.005;

   private final CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
   private final PoseReferenceFrame cameraFrame;
   private final ZUpFrame cameraZUpFrame;

   private final double imagePlaneWidthM;
   private final double imagePlaneHeightM;

   public CPUPerspectiveCameraForTest(double horizontalFOV, double verticalFOV, int pixelWidth, int pixelHeight)
   {
      cameraFrame = new PoseReferenceFrame("cameraFrame", ReferenceFrame.getWorldFrame());
      cameraZUpFrame = new ZUpFrame(cameraFrame, "cameraZupFrame");

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
      double fx = (pixelWidth -  cx) * Math.sin(horizontalFOV / 2.0);
      double fy = (pixelHeight - cy) * Math.sin(verticalFOV / 2.0);


      double widthDensity = pixelWidth / imagePlaneWidthM;
      double heightDensity = pixelWidth / imagePlaneWidthM;
      double fxAlt = focalLength * widthDensity;
      double fyAlt = focalLength * heightDensity;

      if (!MathTools.epsilonEquals(fyAlt, heightDensity / widthDensity * fxAlt, 1e-4))
         throw new RuntimeException("Failed to compute correct parameters");

      cameraIntrinsics.setFx(fxAlt);
      cameraIntrinsics.setFy(fyAlt);
   }

   public void destroy()
   {
      cameraFrame.remove();
      cameraZUpFrame.remove();
   }

   public void setCameraPose(FramePose3DReadOnly cameraPose)
   {
      cameraFrame.setPoseAndUpdate(cameraPose);
   }

   public CameraIntrinsics getCameraIntrinsics()
   {
      return cameraIntrinsics;
   }

   public ReferenceFrame getCameraFrame()
   {
      return cameraFrame;
   }

   public ReferenceFrame getCameraZUpFrame()
   {
      return cameraZUpFrame;
   }

   public Vector3DReadOnly computeDirectionOfPixelFromFocalPoint(int u, int v)
   {
      if (u < 0 || u >= cameraIntrinsics.getWidth())
         throw new RuntimeException("u " + u + " is out of bounds");
      if (v < 0 || v >= cameraIntrinsics.getHeight())
         throw new RuntimeException("v " + v + " is out of bounds");

      // assumes a depth measurement of 1 to get x and y
      double planeXDirection = (u - cameraIntrinsics.getCx()) / cameraIntrinsics.getFx() * focalLength;
      double planeYDirection = (v - cameraIntrinsics.getCy()) / cameraIntrinsics.getFy() * focalLength;

      // length is X, Image X is -Y, Image Y is -Z
      Vector3D direction = new Vector3D(focalLength, -planeXDirection, -planeYDirection);
      direction.normalize();
      return direction;
   }

   public Point3DReadOnly backProject(int u, int v, double z)
   {
      // this maps to x and y coordinates in the image frame
      double x = (u - cameraIntrinsics.getCx()) / cameraIntrinsics.getFx() * z;
      double y = (v - cameraIntrinsics.getCy()) / cameraIntrinsics.getFy() * z;

      // Change to camera frame where x is forward.
      return new Point3D(z, -x, -y);
   }

   public int[] project(Point3DReadOnly pointInCameraFrame)
   {
      double imageX = -pointInCameraFrame.getY();
      double imageY = -pointInCameraFrame.getZ();
      double imageZ = pointInCameraFrame.getX();
      double u = imageX / imageZ * cameraIntrinsics.getFx() + cameraIntrinsics.getCx();
      double v = imageY / imageZ * cameraIntrinsics.getFy() + cameraIntrinsics.getCy();

      return new int[]{(int) u, (int)  v};
   }

}
