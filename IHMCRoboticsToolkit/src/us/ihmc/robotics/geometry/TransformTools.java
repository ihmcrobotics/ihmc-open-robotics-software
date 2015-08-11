package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class TransformTools
{
   public static Quat4d getTransformedQuat(Quat4d quat4d, RigidBodyTransform transform3D)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FrameOrientation start = new FrameOrientation(starting, quat4d);
      start.changeFrame(ending);

      return start.getQuaternionCopy();
   }

   public static Point3d getTransformedPoint(Point3d point3d, RigidBodyTransform transform3D)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FramePoint framePoint = new FramePoint(starting, point3d);
      framePoint.changeFrame(ending);

      return framePoint.getPoint();
   }

   public static Vector3d getTransformedVector(Vector3d vector3d, RigidBodyTransform transform3D)
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
      t.setTranslationAndIdentityRotation(new Vector3d(4.5, 6.6, 22));
      System.out.println(getTransformedPoint(new Point3d(0.0, 1.0, 4.6), t));
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
      
      ret.set(0,0,(t2.mat00 - t1.mat00) / dt);
      ret.set(0,1,(t2.mat01 - t1.mat01) / dt);
      ret.set(0,2,(t2.mat02 - t1.mat02) / dt);
      ret.set(0,3,(t2.mat03 - t1.mat03) / dt);
      ret.set(1,0,(t2.mat10 - t1.mat10) / dt);
      ret.set(1,1,(t2.mat11 - t1.mat11) / dt);
      ret.set(1,2,(t2.mat12 - t1.mat12) / dt);
      ret.set(1,3,(t2.mat13 - t1.mat13) / dt);
      ret.set(2,0,(t2.mat20 - t1.mat20) / dt);
      ret.set(2,1,(t2.mat21 - t1.mat21) / dt);
      ret.set(2,2,(t2.mat22 - t1.mat22) / dt);
      ret.set(2,3,(t2.mat23 - t1.mat23) / dt);
      ret.set(3,0,0);
      ret.set(3,1,0);
      ret.set(3,2,0);
      ret.set(3,3,1);;
      
      return ret;
   }

   public static RigidBodyTransform yawPitchDegreesTransform(Vector3d center, double yawCCWDegrees, double pitchDownDegrees)
   {
      RigidBodyTransform location = new RigidBodyTransform();
      location.rotZ(Math.toRadians(yawCCWDegrees));

      RigidBodyTransform tilt = new RigidBodyTransform();
      tilt.rotY(Math.toRadians(pitchDownDegrees));
      location.multiply(tilt);

      location.setTranslation(center);

      return location;
   }

   public static RigidBodyTransform createTranslationTransform(double x, double y, double z)
   {
      return createTranslationTransform(new Vector3d(x, y, z));
   }

   public static RigidBodyTransform createTranslationTransform(Vector3d translation)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(translation);

      return transform;
   }

   public static RigidBodyTransform createTransformFromTranslationAndEulerAngles(double x, double y, double z, double roll, double pitch, double yaw)
   {
      return createTransformFromTranslationAndEulerAngles(new Vector3d(x, y, z), new Vector3d(roll, pitch, yaw));
   }

   public static RigidBodyTransform createTransformFromTranslationAndEulerAngles(Vector3d translation, Vector3d eulerAngles)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setEuler(eulerAngles);
      transform.setTranslation(translation);

      return transform;
   }

   public static RigidBodyTransform transformLocalZ(RigidBodyTransform originalTransform, double magnitude)
   {
      RigidBodyTransform transform = new RigidBodyTransform(originalTransform);

      Vector3d localZTranslation = new Vector3d(0.0, 0.0, magnitude);
      RigidBodyTransform postTranslation = new RigidBodyTransform();
      postTranslation.setTranslationAndIdentityRotation(localZTranslation);
      transform.multiply(postTranslation);

      return transform;
   }

   public static RigidBodyTransform transformLocalY(RigidBodyTransform originalTransform, double magnitude)
   {
      RigidBodyTransform transform = new RigidBodyTransform(originalTransform);

      Vector3d localTranslation = new Vector3d(0.0, magnitude, 0.0);
      RigidBodyTransform postTranslation = new RigidBodyTransform();
      postTranslation.setTranslationAndIdentityRotation(localTranslation);
      transform.multiply(postTranslation);

      return transform;
   }

   public static RigidBodyTransform transformLocalX(RigidBodyTransform originalTransform, double magnitude)
   {
      RigidBodyTransform transform = new RigidBodyTransform(originalTransform);

      Vector3d localTranslation = new Vector3d(magnitude, 0.0, 0.0);
      RigidBodyTransform postTranslation = new RigidBodyTransform();
      postTranslation.setTranslationAndIdentityRotation(localTranslation);
      transform.multiply(postTranslation);

      return transform;
   }

   public static RigidBodyTransform transformLocal(RigidBodyTransform originalTransform, double localX, double localY, double localZ)
   {
      RigidBodyTransform transform = new RigidBodyTransform(originalTransform);

      Vector3d localTranslation = new Vector3d(localX, localY, localZ);
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
         rotator.rotX(angle);
      }
      else if (axis == Axis.Y)
      {
         rotator.rotY(angle);
      }
      else if (axis == Axis.Z)
      {
         rotator.rotZ(angle);
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
      AxisAngle4d axisAngle4d = new AxisAngle4d();
      rigidBodyTransform.getRotation(axisAngle4d);
      return Math.abs(axisAngle4d.getAngle());
   }
   
   public static double getMagnitudeOfTranslation(RigidBodyTransform rigidBodyTransform)
   {
      Vector3d vector3d = new Vector3d();
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
