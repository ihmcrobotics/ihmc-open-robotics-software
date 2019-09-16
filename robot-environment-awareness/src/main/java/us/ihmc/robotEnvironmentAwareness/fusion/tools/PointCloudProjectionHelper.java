package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.robotEnvironmentAwareness.fusion.MultisenseInformation;

public class PointCloudProjectionHelper
{
   private static final int defaultOffsetU = 11;
   private static final int defaultOffsetV = 0;

   public static final IntrinsicParameters multisenseOnCartIntrinsicParameters = MultisenseInformation.CART.getIntrinsicParameters();
   static
   {
          multisenseOnCartIntrinsicParameters.setFx(566.8350830078125);
          multisenseOnCartIntrinsicParameters.setFy(566.8350830078125);
          multisenseOnCartIntrinsicParameters.setCx(505.5);
          multisenseOnCartIntrinsicParameters.setCy(260.5);
   }

   /**
    * This method is to pack a pixel value (u, v) that is a point projected onto image.
    * The author recommends to use parameter set of 'P' which placed in `CameraInfo` of Multisense.
    */
   public static void projectMultisensePointCloudOnImage(Point3DBasics point, Point2DBasics pixel, IntrinsicParameters param)
   {
      int[] pixelArray = new int[2];
      pixelArray = projectMultisensePointCloudOnImage(point, param);

      pixel.set(pixelArray[0], pixelArray[1]);
   }

   public static void projectMultisensePointCloudOnImage(Point3DBasics point, Point2DBasics pixel)
   {
      projectMultisensePointCloudOnImage(point, pixel, multisenseOnCartIntrinsicParameters, defaultOffsetU, defaultOffsetV);
   }

   public static void projectMultisensePointCloudOnImage(Point3DBasics point, Point2DBasics pixel, int offsetU, int offsetV)
   {
      projectMultisensePointCloudOnImage(point, pixel, multisenseOnCartIntrinsicParameters, offsetU, offsetV);
   }

   public static void projectMultisensePointCloudOnImage(Point3DBasics point, Point2DBasics pixel, IntrinsicParameters param, int offsetU, int offsetV)
   {
      projectMultisensePointCloudOnImage(point, pixel, param);
      pixel.add(offsetU, offsetV);
   }

   /**
    * Point cloud projection from camera transform which is not same with world frame.
    */
   public static int[] projectMultisensePointCloudOnImage(Point3DBasics point, IntrinsicParameters param, Point3DBasics cameraPosition,
                                                          QuaternionBasics cameraOrientation)
   {
      Point3D pointToCamera = new Point3D(point);
      RigidBodyTransform transformWorldToCamera = new RigidBodyTransform(cameraOrientation, cameraPosition);
      pointToCamera.applyInverseTransform(transformWorldToCamera);
      return projectMultisensePointCloudOnImage(pointToCamera, param);
   }

   public static int[] projectMultisensePointCloudOnImage(Point3DBasics point, IntrinsicParameters param)
   {
      double fx = param.getFx();
      double fy = param.getFy();
      double cx = param.getCx();
      double cy = param.getCy();

      double cameraX = -point.getY();
      double cameraY = -point.getZ();
      double cameraZ = point.getX();

      double normX = cameraX / cameraZ;
      double normY = cameraY / cameraZ;

      int u = (int) (fx * normX + cx);
      int v = (int) (fy * normY + cy);

      int[] pixel = new int[2];
      pixel[0] = u;
      pixel[1] = v;

      return pixel;
   }
}