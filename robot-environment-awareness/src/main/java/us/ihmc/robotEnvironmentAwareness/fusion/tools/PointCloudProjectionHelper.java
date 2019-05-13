package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public class PointCloudProjectionHelper
{
   public static final IntrinsicParameters multisenseOnCartIntrinsicParameters = new IntrinsicParameters();
   static
   {
      multisenseOnCartIntrinsicParameters.setFx(601.5020141601562);
      multisenseOnCartIntrinsicParameters.setFy(602.0339965820312);
      multisenseOnCartIntrinsicParameters.setCx(520.92041015625);
      multisenseOnCartIntrinsicParameters.setCy(273.5399169921875);
   }

   /**
    * This method is to pack a pixel value (u, v) that is a point projected onto image.
    * The author recommends to use parameter set of 'P' which placed in `CameraInfo` of Multisense.
    */
   public static void projectMultisensePointCloudOnImage(Point3DBasics point, Point2DBasics pixel, IntrinsicParameters param)
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

      pixel.set(u, v);
   }

   public static void projectMultisensePointCloudOnImage(Point3DBasics point, Point2DBasics pixel)
   {
      projectMultisensePointCloudOnImage(point, pixel, multisenseOnCartIntrinsicParameters);
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
}