package us.ihmc.robotics.geometry;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class TransformTools
{
   /**
    * Creates a transform that transforms to the given point and rotates to make the z axis align
    * with the normal vector.
    */
   public static RigidBodyTransform createTransformFromPointAndZAxis(FramePoint3D point, FrameVector3D zAxis)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      ret.setRotation(EuclidGeometryTools.axisAngleFromZUpToVector3D(zAxis.getVector()));
      ret.setTranslation(point.getPoint());
      return ret;
   }

   public static RigidBodyTransform yawPitchDegreesTransform(Vector3D center, double yawCCWDegrees, double pitchDownDegrees)
   {
      RigidBodyTransform location = new RigidBodyTransform();
      location.setRotationYawPitchRoll(Math.toRadians(yawCCWDegrees), Math.toRadians(pitchDownDegrees), 0.0);
      location.setTranslation(center);
      return location;
   }

   public static RigidBodyTransform createTranslationTransform(double x, double y, double z)
   {
      return createTranslationTransform(new Vector3D(x, y, z));
   }

   public static RigidBodyTransform createTranslationTransform(Vector3D translation)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(translation);
      return transform;
   }

   public static RigidBodyTransform createTransformFromTranslationAndEulerAngles(double x, double y, double z, double roll, double pitch, double yaw)
   {
      return createTransformFromTranslationAndEulerAngles(new Vector3D(x, y, z), new Vector3D(roll, pitch, yaw));
   }

   public static RigidBodyTransform createTransformFromTranslationAndEulerAngles(Vector3D translation, Vector3D eulerAngles)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationEuler(eulerAngles);
      transform.setTranslation(translation);
      return transform;
   }

   public static void appendRotation(RigidBodyTransform transformToModify, double angle, Axis axis)
   {
      switch (axis)
      {
      case X:
         transformToModify.appendRollRotation(angle);
         break;
      case Y:
         transformToModify.appendPitchRotation(angle);
         break;
      case Z:
         transformToModify.appendYawRotation(angle);
         break;
      default:
         throw new RuntimeException("Unhandled value of Axis: " + axis);
      }
   }

   public static void appendRotation(AffineTransform transformToModify, double angle, Axis axis)
   {
      switch (axis)
      {
      case X:
         transformToModify.appendRollRotation(angle);
         break;
      case Y:
         transformToModify.appendPitchRotation(angle);
         break;
      case Z:
         transformToModify.appendYawRotation(angle);
         break;
      default:
         throw new RuntimeException("Unhandled value of Axis: " + axis);
      }
   }

   public static RigidBodyTransform getTransformFromA2toA1(RigidBodyTransform transformFromBtoA1, RigidBodyTransform transformFromBtoA2)
   {
      RigidBodyTransform ret = new RigidBodyTransform(transformFromBtoA1);
      ret.multiplyInvertOther(transformFromBtoA2);
      return ret;
   }

   public static double getMagnitudeOfAngleOfRotation(RigidBodyTransform rigidBodyTransform)
   {
      AxisAngle axisAngle = new AxisAngle();
      rigidBodyTransform.getRotation(axisAngle);
      return Math.abs(axisAngle.getAngle());
   }

   public static double getMagnitudeOfTranslation(RigidBodyTransform rigidBodyTransform)
   {
      return rigidBodyTransform.getTranslationVector().length();
   }

   public static double getSizeOfTransformWithRotationScaled(RigidBodyTransform rigidBodyTransform, double radiusForScalingRotation)
   {
      return getMagnitudeOfTranslation(rigidBodyTransform) + radiusForScalingRotation * getMagnitudeOfAngleOfRotation(rigidBodyTransform);
   }

   /** This will get the magnitude of the transform between two transforms that are relative to the same frame.
    * @param rigidBodyTransform1 with respect to frame B
    * @param rigidBodyTransform2 with respect to frame B
    * @param radiusForScalingRotation
    * @return
    */
   public static double getSizeOfTransformBetweenTwoWithRotationScaled(RigidBodyTransform rigidBodyTransform1, RigidBodyTransform rigidBodyTransform2,
                                                                       double radiusForScalingRotation)
   {
      RigidBodyTransform temp = getTransformFromA2toA1(rigidBodyTransform1, rigidBodyTransform2);
      return getMagnitudeOfTranslation(temp) + radiusForScalingRotation * getMagnitudeOfAngleOfRotation(temp);
   }
}
