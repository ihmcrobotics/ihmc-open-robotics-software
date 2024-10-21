package us.ihmc.commons.robotics;

import us.ihmc.commons.AngleTools;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class HeadingAngleTools
{
   public static double angleMinusPiToPi(Vector2DReadOnly startVector, Vector2DReadOnly endVector)
   {
      double absoluteAngle = Math.acos(startVector.dot(endVector) / startVector.norm() / endVector.norm());

      Vector3D start3d = new Vector3D(startVector.getX(), startVector.getY(), 0.0);
      Vector3D end3d = new Vector3D(endVector.getX(), endVector.getY(), 0.0);

      Vector3D crossProduct = new Vector3D();
      crossProduct.cross(start3d, end3d);

      if (crossProduct.getZ() >= 0.0)
      {
         return absoluteAngle;
      }
      else
      {
         return -absoluteAngle;
      }
   }

   /**
    * Returns an angle between two points + heading Offset from -PI to PI.
    * If the x or y components are both under the noTranslationTolerance,
    * then the initial orientation as given in startPose will be returned.
    *
    * @param startPose              initial position and orientation
    * @param endPoint               end position
    * @param headingOffset          offset from path angle
    * @param noTranslationTolerance tolerance for determining if path angle should be determined
    * @return number between -PI and PI
    */
   public static double calculateHeading(FramePose2DReadOnly startPose, FramePoint2DReadOnly endPoint, double headingOffset, double noTranslationTolerance)
   {
      double deltaX = endPoint.getX() - startPose.getX();
      double deltaY = endPoint.getY() - startPose.getY();
      double heading;
      if (Math.abs(deltaX) < noTranslationTolerance && Math.abs(deltaY) < noTranslationTolerance)
      {
         heading = startPose.getYaw();
      }
      else
      {
         double pathHeading = Math.atan2(deltaY, deltaX);
         heading = AngleTools.trimAngleMinusPiToPi(pathHeading + headingOffset);
      }
      return heading;
   }

   /**
    * Returns an angle between two points + heading Offset from -PI to PI.
    *
    * @param startPose              initial position and orientation
    * @param endPoint               end position
    * @param headingOffset          offset from path angle
    * @return number between -PI and PI
    */
   public static double calculateHeading(Tuple3DReadOnly startPose, Tuple3DReadOnly endPoint, double headingOffset)
   {
      double deltaX = endPoint.getX() - startPose.getX();
      double deltaY = endPoint.getY() - startPose.getY();

      double pathHeading = Math.atan2(deltaY, deltaX);
      return AngleTools.trimAngleMinusPiToPi(pathHeading + headingOffset);
   }

   /**
    * Returns an angle between two points + heading Offset from -PI to PI.
    *
    * @param startPose initial position
    * @param endPoint  end position
    * @return number between -PI and PI
    */
   public static double calculateHeading(Point2DReadOnly startPose, Point2DReadOnly endPoint)
   {
      double deltaX = endPoint.getX() - startPose.getX();
      double deltaY = endPoint.getY() - startPose.getY();
      double heading;

      double pathHeading = Math.atan2(deltaY, deltaX);
      heading = AngleTools.trimAngleMinusPiToPi(pathHeading);

      return heading;
   }

   public static void roundToGivenPrecisionForAngles(Tuple3DBasics tuple3d, double precision)
   {
      tuple3d.setX(AngleTools.roundToGivenPrecisionForAngle(tuple3d.getX(), precision));
      tuple3d.setY(AngleTools.roundToGivenPrecisionForAngle(tuple3d.getY(), precision));
      tuple3d.setZ(AngleTools.roundToGivenPrecisionForAngle(tuple3d.getZ(), precision));
   }
}
