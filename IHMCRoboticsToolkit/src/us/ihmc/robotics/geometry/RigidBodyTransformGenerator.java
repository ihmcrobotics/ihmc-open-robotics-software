package us.ihmc.robotics.geometry;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;

/**
 * Class for intuitively generating RigidBodyTransforms by translating and rotating
 * starting at the to frame and getting to the from frame. Removes the burden from
 * the user of thinking about transform multiplication ordering etc. Instead, the user
 * just needs to think about a reference frame that is alternately translated and rotated
 * with each additional operation happening in the coordinate system of the frame as it
 * currently is. For example rotating 90 degrees about x will then make motion in y be
 * motion in the original z.
 */
public class RigidBodyTransformGenerator
{
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public RigidBodyTransformGenerator()
   {
   }

   public RigidBodyTransformGenerator(RigidBodyTransform transform)
   {
      setTransform(transform);
   }

   public RigidBodyTransformGenerator(RigidBodyTransformGenerator generator)
   {
      setTransform(generator.transform);
   }

   public void set(RigidBodyTransformGenerator transformGenerator)
   {
      this.transform.set(transformGenerator.transform);
   }

   public void setTransform(RigidBodyTransform transform)
   {
      this.transform.set(transform);
   }

   public RigidBodyTransform getRigidBodyTransformCopy()
   {
      return new RigidBodyTransform(transform);
   }

   public void getRigidyBodyTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transform);
   }

   public void identity()
   {
      transform.setIdentity();
   }

   public void translate(double x, double y, double z)
   {
      tempTransform.setTranslationAndIdentityRotation(x, y, z);
      transform.multiply(transform, tempTransform);
   }

   public void translate(Vector3d translationVector)
   {
      tempTransform.setTranslationAndIdentityRotation(translationVector);
      transform.multiply(transform, tempTransform);
   }

   public void rotate(double rotationAngle, Axis axis)
   {
      switch (axis)
      {
      case X:
      {
         tempTransform.setRotationEulerAndZeroTranslation(rotationAngle, 0.0, 0.0);
         break;
      }
      case Y:
      {
         tempTransform.setRotationEulerAndZeroTranslation(0.0, rotationAngle, 0.0);
         break;
      }
      case Z:
      {
         tempTransform.setRotationEulerAndZeroTranslation(0.0, 0.0, rotationAngle);
         break;
      }

      }
      transform.multiply(transform, tempTransform);
   }

   public void rotateEuler(double rotateX, double rotateY, double rotateZ)
   {
      tempTransform.setRotationEulerAndZeroTranslation(rotateX, rotateY, rotateZ);
      transform.multiply(transform, tempTransform);
   }

   public void rotateEuler(Vector3d eulerAngles)
   {
      tempTransform.setRotationEulerAndZeroTranslation(eulerAngles);
      transform.multiply(transform, tempTransform);
   }

   public void rotate(Matrix3d rotationMatrix)
   {
      tempTransform.setRotationAndZeroTranslation(rotationMatrix);
      transform.multiply(transform, tempTransform);
   }

   public void rotate(Quat4d rotationQuaternion)
   {
      tempTransform.setRotationAndZeroTranslation(rotationQuaternion);
      transform.multiply(transform, tempTransform);
   }

   public void translateThenRotate(RigidBodyTransform translateThenRotateTransform)
   {
      tempTransform.set(translateThenRotateTransform);
      transform.multiply(transform, tempTransform);
   }

   public void translateThenRotateEuler(Vector3d translationVector, Vector3d eulerAngles)
   {
      tempTransform.setRotationEulerAndZeroTranslation(eulerAngles);
      tempTransform.setTranslation(translationVector);
      transform.multiply(transform, tempTransform);
   }

}
