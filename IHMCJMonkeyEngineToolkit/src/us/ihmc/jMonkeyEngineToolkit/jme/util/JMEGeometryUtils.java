package us.ihmc.jMonkeyEngineToolkit.jme.util;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

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

   public static RigidBodyTransform transformFromJMECoordinatesToZup(Transform transform)
   {
      RigidBodyTransform modifiedTransform = new RigidBodyTransform(yUpToZupTransform);
      RigidBodyTransform temp = jmeTransformToTransform3D(transform);
      modifiedTransform.multiply(temp);

      return modifiedTransform;
   }

   public static Transform transformFromZupToJMECoordinates(RigidBodyTransform transform)
   {
      RigidBodyTransform modifiedTransform = new RigidBodyTransform(zUpToYupTransform);
      modifiedTransform.multiply(transform);

      return j3dTransform3DToJMETransform( modifiedTransform );
   }
   
  /*
   * 
   */
   public static void transform(Transform matrixToModify, Transform transformToApply)
   {
      matrixToModify.combineWithParent(transformToApply);
   }
   
   public static Transform multiply(Transform matrix, Transform transformToApply)
   {
      Transform temp = new Transform();
      temp.set(matrix);
      temp.combineWithParent(transformToApply);
      return temp;
   }
   
   public static void multiply(Transform matrix, Transform transformToApply, Transform result)
   {
      result.set(matrix);
      result.combineWithParent(transformToApply);
   }
   
/*   public static Transform multiply(Transform a, Transform b)
   {
      RigidBodyTransform A = jmeTransformToTransform3D(a);
      RigidBodyTransform B = jmeTransformToTransform3D(b);
      A.multiply(B);
      return j3dTransform3DToJMETransform( A );
   }*/

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
   
   /// under no circumstances change this to public. This method is dangerous and misleading.
   private static Transform j3dTransform3DToJMETransform(RigidBodyTransform transform3D)
   {
      javax.vecmath.Quat4f   quat   = new javax.vecmath.Quat4f();
      javax.vecmath.Vector3f vector = new javax.vecmath.Vector3f();
      transform3D.get(quat, vector);
      Vector3f jmeVector = new Vector3f(vector.getX(), vector.getY(), vector.getZ());
      Quaternion jmeQuat = new Quaternion(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
      Transform ret = new Transform(jmeVector, jmeQuat, new Vector3f(1.0f, 1.0f, 1.0f));

      return ret;
   }

   /// under no circumstances change this to public. This method is dangerous and misleading.
   private static RigidBodyTransform jmeTransformToTransform3D(Transform jmeTransform)
   {
      Quaternion jmeQuat = jmeTransform.getRotation();
      Vector3f jmeVect = jmeTransform.getTranslation();
      javax.vecmath.Quat4d quat = new  javax.vecmath.Quat4d(jmeQuat.getX(), jmeQuat.getY(), jmeQuat.getZ(), jmeQuat.getW());
      Vector3d vect = new Vector3d(jmeVect.getX(), jmeVect.getY(), jmeVect.getZ());
      RigidBodyTransform ret = new RigidBodyTransform(quat, vect);
      return ret;
   }

   public static Transform getInverse(Transform transform)
   {
      RigidBodyTransform transform3D = jmeTransformToTransform3D(transform);
      transform3D.invert();
      Transform ret = j3dTransform3DToJMETransform(transform3D);

      return ret;
   }
   
   public static Transform invert(Transform transform)
   {
      RigidBodyTransform transform3D = jmeTransformToTransform3D(transform);
      transform3D.invert();
      return transform.set(  j3dTransform3DToJMETransform(transform3D ) );
   }
   
   static public boolean epsilonEquals(Transform a, Transform b, double epsilon)
   {
      RigidBodyTransform A = jmeTransformToTransform3D(a);
      RigidBodyTransform B = jmeTransformToTransform3D(b);
      return A.epsilonEquals(B, epsilon);    
   }

   public static void main(String[] args)
   {
   }

}
