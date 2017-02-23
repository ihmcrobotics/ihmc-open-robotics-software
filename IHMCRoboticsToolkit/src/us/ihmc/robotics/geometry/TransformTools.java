package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class TransformTools
{
   public static Quaternion getTransformedQuat(Quaternion quat4d, RigidBodyTransform transform3D)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FrameOrientation start = new FrameOrientation(starting, quat4d);
      start.changeFrame(ending);

      return start.getQuaternionCopy();
   }

   public static Point3D getTransformedPoint(Point3D point3d, RigidBodyTransform transform3D)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FramePoint framePoint = new FramePoint(starting, point3d);
      framePoint.changeFrame(ending);

      return framePoint.getPoint();
   }

   public static Vector3D getTransformedVector(Vector3D vector3d, RigidBodyTransform transform3D)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FrameVector framePoint = new FrameVector(starting, vector3d);
      framePoint.changeFrame(ending);

      return framePoint.getVector();
   }

   public static void main(String[] args)
   {
      RigidBodyTransform t = new RigidBodyTransform();
      t.setTranslationAndIdentityRotation(new Vector3D(4.5, 6.6, 22));
      System.out.println(getTransformedPoint(new Point3D(0.0, 1.0, 4.6), t));
   }

   public static double getMaxLInfiniteDistance(RigidBodyTransform t1, RigidBodyTransform t2)
   {
      double[] t1Matrix = new double[16];
      t1.get(t1Matrix);

      double[] t2Matrix = new double[16];
      t2.get(t2Matrix);

      double maxDifference = 0.0;

      for (int i = 0; i < 16; i++)
      {
         double difference = t1Matrix[i] - t2Matrix[i];
         if (Math.abs(difference) > maxDifference)
            maxDifference = Math.abs(difference);
      }

      return maxDifference;
   }
   
   public static DenseMatrix64F differentiate(RigidBodyTransform t1, RigidBodyTransform t2, double dt)
   {
      DenseMatrix64F ret = new DenseMatrix64F(4,4);
      
      ret.set(0,0,(t2.getM00() - t1.getM00()) / dt);
      ret.set(0,1,(t2.getM01() - t1.getM01()) / dt);
      ret.set(0,2,(t2.getM02() - t1.getM02()) / dt);
      ret.set(0,3,(t2.getM03() - t1.getM03()) / dt);
      ret.set(1,0,(t2.getM10() - t1.getM10()) / dt);
      ret.set(1,1,(t2.getM11() - t1.getM11()) / dt);
      ret.set(1,2,(t2.getM12() - t1.getM12()) / dt);
      ret.set(1,3,(t2.getM13() - t1.getM13()) / dt);
      ret.set(2,0,(t2.getM20() - t1.getM20()) / dt);
      ret.set(2,1,(t2.getM21() - t1.getM21()) / dt);
      ret.set(2,2,(t2.getM22() - t1.getM22()) / dt);
      ret.set(2,3,(t2.getM23() - t1.getM23()) / dt);
      ret.set(3,0,0);
      ret.set(3,1,0);
      ret.set(3,2,0);
      ret.set(3,3,1);;
      
      return ret;
   }

   public static RigidBodyTransform yawPitchDegreesTransform(Vector3D center, double yawCCWDegrees, double pitchDownDegrees)
   {
      RigidBodyTransform location = new RigidBodyTransform();
      location.setRotationYawAndZeroTranslation(Math.toRadians(yawCCWDegrees));

      RigidBodyTransform tilt = new RigidBodyTransform();
      tilt.setRotationPitchAndZeroTranslation(Math.toRadians(pitchDownDegrees));
      location.multiply(tilt);

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
      transform.setRotationEulerAndZeroTranslation(eulerAngles);
      transform.setTranslation(translation);

      return transform;
   }

   public static RigidBodyTransform transformLocalZ(RigidBodyTransform originalTransform, double magnitude)
   {
      RigidBodyTransform transform = new RigidBodyTransform(originalTransform);

      Vector3D localZTranslation = new Vector3D(0.0, 0.0, magnitude);
      RigidBodyTransform postTranslation = new RigidBodyTransform();
      postTranslation.setTranslationAndIdentityRotation(localZTranslation);
      transform.multiply(postTranslation);

      return transform;
   }

   public static RigidBodyTransform transformLocalY(RigidBodyTransform originalTransform, double magnitude)
   {
      RigidBodyTransform transform = new RigidBodyTransform(originalTransform);

      Vector3D localTranslation = new Vector3D(0.0, magnitude, 0.0);
      RigidBodyTransform postTranslation = new RigidBodyTransform();
      postTranslation.setTranslationAndIdentityRotation(localTranslation);
      transform.multiply(postTranslation);

      return transform;
   }

   public static RigidBodyTransform transformLocalX(RigidBodyTransform originalTransform, double magnitude)
   {
      RigidBodyTransform transform = new RigidBodyTransform(originalTransform);

      Vector3D localTranslation = new Vector3D(magnitude, 0.0, 0.0);
      RigidBodyTransform postTranslation = new RigidBodyTransform();
      postTranslation.setTranslationAndIdentityRotation(localTranslation);
      transform.multiply(postTranslation);

      return transform;
   }

   public static RigidBodyTransform transformLocal(RigidBodyTransform originalTransform, double localX, double localY, double localZ)
   {
      RigidBodyTransform transform = new RigidBodyTransform(originalTransform);

      Vector3D localTranslation = new Vector3D(localX, localY, localZ);
      RigidBodyTransform postTranslation = new RigidBodyTransform();
      postTranslation.setTranslationAndIdentityRotation(localTranslation);
      transform.multiply(postTranslation);

      return transform;
   }

   public static void rotate(RigidBodyTransform transform, double angle, Axis axis)
   {
      RigidBodyTransform rotator = new RigidBodyTransform();

      if (axis == Axis.X)
      {
         rotator.setRotationRollAndZeroTranslation(angle);
      }
      else if (axis == Axis.Y)
      {
         rotator.setRotationPitchAndZeroTranslation(angle);
      }
      else if (axis == Axis.Z)
      {
         rotator.setRotationYawAndZeroTranslation(angle);
      }

      transform.multiply(rotator);
   }

   public static void rotate(AffineTransform transform, double angle, Axis axis)
   {
      RigidBodyTransform rotator = new RigidBodyTransform();

      if (axis == Axis.X)
      {
         rotator.setRotationRollAndZeroTranslation(angle);
      }
      else if (axis == Axis.Y)
      {
         rotator.setRotationPitchAndZeroTranslation(angle);
      }
      else if (axis == Axis.Z)
      {
         rotator.setRotationYawAndZeroTranslation(angle);
      }

      transform.multiply(rotator);
   }
   
   public static RigidBodyTransform getTransformFromA2toA1(RigidBodyTransform transformFromBtoA1, RigidBodyTransform transformFromBtoA2)
   {
      RigidBodyTransform temp = new RigidBodyTransform(transformFromBtoA2);
      temp.invert();
      
      RigidBodyTransform ret = new RigidBodyTransform(transformFromBtoA1);
      
      ret.multiply(temp);
      
      return ret;
   }
   
   public static double getMagnitudeOfAngleOfRotation(RigidBodyTransform rigidBodyTransform)
   {
      AxisAngle axisAngle4d = new AxisAngle();
      rigidBodyTransform.getRotation(axisAngle4d);
      return Math.abs(axisAngle4d.getAngle());
   }
   
   public static double getMagnitudeOfTranslation(RigidBodyTransform rigidBodyTransform)
   {
      Vector3D vector3d = new Vector3D();
      rigidBodyTransform.getTranslation(vector3d);
      return vector3d.length();
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
   public static double getSizeOfTransformBetweenTwoWithRotationScaled(RigidBodyTransform rigidBodyTransform1, RigidBodyTransform rigidBodyTransform2, double radiusForScalingRotation)
   {
      RigidBodyTransform temp = getTransformFromA2toA1(rigidBodyTransform1, rigidBodyTransform2);
      return getMagnitudeOfTranslation(temp) + radiusForScalingRotation * getMagnitudeOfAngleOfRotation(temp);
   }
}
