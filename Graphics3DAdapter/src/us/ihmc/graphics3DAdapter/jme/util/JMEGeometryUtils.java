package us.ihmc.graphics3DAdapter.jme.util;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

public class JMEGeometryUtils
{
   private final static Quaternion zUpToYup = new Quaternion();
   private final static Quaternion yUpToZup = new Quaternion();


   static
   {
      Matrix3f matrix3f = new Matrix3f(0, 1, 0, 0, 0, 1, 1, 0, 0);

      zUpToYup.fromRotationMatrix(matrix3f);
      yUpToZup.set(zUpToYup.inverse());
   }

   public static Quaternion getRotationFromJMEToZupCoordinates()
   {
      return new Quaternion(zUpToYup);
   }

   public static Quaternion getRotationFromZupToJMECoordinates()
   {
      return new Quaternion(yUpToZup);
   }

   private final static RigidBodyTransform zUpToYupTransform = new RigidBodyTransform(JMEDataTypeUtils.jMEQuaternionToVecMathQuat4d(zUpToYup), new Vector3d());
   private final static RigidBodyTransform yUpToZupTransform = new RigidBodyTransform(JMEDataTypeUtils.jMEQuaternionToVecMathQuat4d(yUpToZup), new Vector3d());

   /*
    * Be careful with the multLocal and mult functions. The documentation is not always correct in what arguments are safe to pass
    */

   public static void transformFromZupToJMECoordinates(Vector3f point)
   {
      zUpToYup.multLocal(point);
   }

   public static void transformFromZupToJMECoordinates(Quaternion rotation)
   {
      zUpToYup.mult(rotation, rotation);
   }

   public static void transformFromJMECoordinatesToZup(Vector3f point)
   {
      yUpToZup.multLocal(point);
   }

   public static void transformFromJMECoordinatesToZup(Quaternion rotation)
   {
      yUpToZup.mult(rotation, rotation);
   }

   public static Ray transformRayFromJMECoordinateToZup(Ray jmeRay)
   {
      Vector3f origin = new Vector3f(jmeRay.getOrigin());
      Vector3f direction = new Vector3f(jmeRay.getDirection());

      transformFromJMECoordinatesToZup(origin);
      transformFromJMECoordinatesToZup(direction);

      return new Ray(origin, direction);
   }

   public static Ray transformRayFromZupToJMECoordinate(Ray zupRay)
   {
      Vector3f origin = new Vector3f(zupRay.getOrigin());
      Vector3f direction = new Vector3f(zupRay.getDirection());

      transformFromZupToJMECoordinates(origin);
      transformFromZupToJMECoordinates(direction);

      return new Ray(origin, direction);
   }

   public static RigidBodyTransform transformFromJMECoordinatesToZup(RigidBodyTransform transform)
   {
      RigidBodyTransform modifiedTransform = new RigidBodyTransform(yUpToZupTransform);
      modifiedTransform.multiply(transform);

      return modifiedTransform;
   }

   public static RigidBodyTransform transformFromZupToJMECoordinates(RigidBodyTransform transform)
   {
      RigidBodyTransform modifiedTransform = new RigidBodyTransform(zUpToYupTransform);
      modifiedTransform.multiply(transform);

      return modifiedTransform;
   }

// public static Vector2d projectOntoZupXY(Vector3f vector)
// {
//    Vector3f vectorZup = new Vector3f(vector);
//    JMEGeometryUtils.transformFromJMECoordinatesToZup(vectorZup);
//    Vector2d vectorZup2d = new Vector2d(vectorZup.x, vectorZup.y);
//    return vectorZup2d;
// }
// public static Vector3f unProjectFromZupXY(Vector2d vector)
// {
//    Vector3f vector3f = new Vector3f( (float)vector.x,(float)vector.y,0.0f);
//    transformFromZupToJMECoordinates(vector3f);
//    return vector3f;
// }

   public static Transform getInverse(Transform transform)
   {
      RigidBodyTransform transform3D = JMEDataTypeUtils.jmeTransformToTransform3D(transform);
      transform3D.invert();

      Transform ret = JMEDataTypeUtils.j3dTransform3DToJMETransform(transform3D);

      return ret;
   }

   public static void main(String[] args)
   {
   }

}
