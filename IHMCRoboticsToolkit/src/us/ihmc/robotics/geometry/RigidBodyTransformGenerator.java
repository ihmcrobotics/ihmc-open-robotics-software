package us.ihmc.robotics.geometry;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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
      transform.set(transform);
      transform.multiply(tempTransform);
   }

   public void translate(Vector3D translationVector)
   {
      tempTransform.setTranslationAndIdentityRotation(translationVector);
      transform.set(transform);
      transform.multiply(tempTransform);
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
      transform.set(transform);
      transform.multiply(tempTransform);
   }

   public void rotateEuler(double rotateX, double rotateY, double rotateZ)
   {
      tempTransform.setRotationEulerAndZeroTranslation(rotateX, rotateY, rotateZ);
      transform.set(transform);
      transform.multiply(tempTransform);
   }

   public void rotateEuler(Vector3D eulerAngles)
   {
      tempTransform.setRotationEulerAndZeroTranslation(eulerAngles);
      transform.set(transform);
      transform.multiply(tempTransform);
   }

   public void rotate(RotationMatrix rotationMatrix)
   {
      tempTransform.setRotationAndZeroTranslation(rotationMatrix);
      transform.set(transform);
      transform.multiply(tempTransform);
   }

   public void rotate(Quaternion rotationQuaternion)
   {
      tempTransform.setRotationAndZeroTranslation(rotationQuaternion);
      transform.set(transform);
      transform.multiply(tempTransform);
   }

   public void translateThenRotate(RigidBodyTransform translateThenRotateTransform)
   {
      tempTransform.set(translateThenRotateTransform);
      transform.set(transform);
      transform.multiply(tempTransform);
   }

   public void translateThenRotateEuler(Vector3D translationVector, Vector3D eulerAngles)
   {
      tempTransform.setRotationEulerAndZeroTranslation(eulerAngles);
      tempTransform.setTranslation(translationVector);
      transform.set(transform);
      transform.multiply(tempTransform);
   }

}
