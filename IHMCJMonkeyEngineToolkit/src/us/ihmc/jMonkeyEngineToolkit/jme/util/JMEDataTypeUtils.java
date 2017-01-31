package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.awt.Color;
import java.util.List;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.TexCoord2f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;

import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Transform;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.Ray3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class JMEDataTypeUtils
{
   public static Point3d jmeVector3fToJ3DPoint3d(Vector3f original)
   {
      return new Point3d(original.getX(), original.getY(), original.getZ());
   }

   public static Vector3d jmeVector3fToVecmathVector3d(Vector3f original)
   {
      return new Vector3d(original.getX(), original.getY(), original.getZ());
   }

   public static float[] quaternionToEuler(Quaternion quat)
   {
      float pitch = (float) Math.atan2(2 * (quat.getW() * quat.getX() + quat.getY() * quat.getZ()),
            (1 - 2 * (quat.getX() * quat.getX() + quat.getY() * quat.getY())));
      float roll = (float) Math.asin(2 * (quat.getW() * quat.getY() - quat.getZ() * quat.getX()));
      float yaw = (float) Math.atan2(2 * (quat.getW() * quat.getZ() + quat.getY() * quat.getX()),
            (1 - 2 * (quat.getY() * quat.getY() + quat.getZ() * quat.getZ())));

      yaw = yaw * -1;

      if (yaw < 0)
      {
         yaw = (float) (Math.PI * 2 + yaw);
      }

      //    System.out.println("Pitch: " + pitch);
      //    System.out.println("Roll: " + roll);
      //      System.out.println("Yaw: " + Math.toDegrees(yaw));

      float[] angles = new float[3];
      angles[0] = pitch;
      angles[1] = roll;
      angles[2] = yaw;

      return angles;

   }

   public static Vector3f vecMathTuple3dToJMEVector3f(Tuple3d original)
   {
      Vector3f target = new Vector3f();
      packVecMathTuple3dInJMEVector3f(original, target);

      return target;
   }

   public static Quaternion vecMathQuat4dToJMEQuaternion(Quat4d original)
   {
      Quaternion target = new Quaternion();
      packVectMathQuat4dInJMEQuaternion(original, target);

      return target;
   }

   public static Quat4d jMEQuaternionToVecMathQuat4d(Quaternion original)
   {
      Quat4d target = new Quat4d();
      packJMEQuaterionInVecMathQuat4d(original, target);

      return target;
   }

   public static void packJMEVector3fInVecMathTuple3d(Vector3f original, Tuple3d target)
   {
      target.set(original.getX(), original.getY(), original.getZ());
   }

   public static void packJMEQuaterionInVecMathQuat4d(Quaternion original, Quat4d target)
   {
      target.set(original.getX(), original.getY(), original.getZ(), original.getW());

      // do not remove the normalization. 
      // The conversion from float to double generates very tiny differences which make the 
      // quaternion SLIGHTLY not normal.

      target.normalize();
   }

   public static void packVecMathTuple3dInJMEVector3f(Tuple3d original, Vector3f target)
   {
      target.set((float) original.getX(), (float) original.getY(), (float) original.getZ());
   }

   public static void packVecMathTuple3fInJMEVector3f(Tuple3f original, Vector3f target)
   {
      target.set(original.getX(), original.getY(), original.getZ());
   }

   public static void packVectMathQuat4dInJMEQuaternion(Quat4d original, Quaternion target)
   {
      target.set((float) original.getX(), (float) original.getY(), (float) original.getZ(), (float) original.getW());
   }

   public static void packFramePoseInJMEVector(FramePose original, Vector3f target)
   {
      target.set((float) original.getX(), (float) original.getY(), (float) original.getZ());
   }

   public static void packFramePointInJMEVector(FramePoint original, Vector3f target)
   {
      target.set((float) original.getX(), (float) original.getY(), (float) original.getZ());
   }

   public static void packFrameOrientationInJMEQuaternion(FrameOrientation original, Quaternion target)
   {
      Quat4d quat4d = new Quat4d();
      packVectMathQuat4dInJMEQuaternion(original.getQuaternion(), target);
   }

   public static void packFramePoseInJMEQuaternion(FramePose original, Quaternion target)
   {
      Quat4d quat4d = new Quat4d();
      original.getOrientation(quat4d);
      packVectMathQuat4dInJMEQuaternion(quat4d, target);
   }

   public static void packFramePoseInJMEQuaternionAndVector(FramePose original, Vector3f targetVector, Quaternion targetQuaternion)
   {
      packFramePoseInJMEVector(original, targetVector);
      packFramePoseInJMEQuaternion(original, targetQuaternion);
   }

   public static ColorRGBA colorToColorRGBA(Color color)
   {
      return new ColorRGBA(color.getRed() / 255.0f, color.getGreen() / 255.0f, color.getBlue() / 255.0f, color.getAlpha() / 255.0f);
   }

   public static ColorRGBA jMEColorRGBAFromVecMathColor3f(Color3f originalColor, double alpha)
   {
      float r = originalColor.getX();
      float g = originalColor.getY();
      float b = originalColor.getZ();

      return new ColorRGBA(r, g, b, (float) alpha);
   }

   public static Vector3f[] vecMathTuple3fArrayToJMEVector3fArray(Tuple3f[] original)
   {
      Vector3f[] ret = new Vector3f[original.length];
      for (int i = 0; i < original.length; i++)
      {
         ret[i] = new Vector3f();
         packVecMathTuple3fInJMEVector3f(original[i], ret[i]);
      }

      return ret;
   }

   public static Ray ray3dToJMERay(Ray3d ray)
   {
      return new Ray(vecMathTuple3dToJMEVector3f(ray.getPoint()), vecMathTuple3dToJMEVector3f(ray.getVector()));
   }

   public static Ray3d jmeRayToRay3d(Ray ray)
   {
      return new Ray3d(jmeVector3fToJ3DPoint3d(ray.getOrigin()), jmeVector3fToVecmathVector3d(ray.getDirection()));
   }

   public static Vector2f[] texCoord2fArrayToJMEVector2fArray(TexCoord2f[] texCoords)
   {
      if (texCoords == null)
         return null;

      Vector2f[] vectors = new Vector2f[texCoords.length];

      for (int i = 0; i < texCoords.length; i++)
      {
         vectors[i] = new Vector2f();
         packTexCoord2fInJMEVector2f(texCoords[i], vectors[i]);
      }

      return vectors;
   }

   public static void packTexCoord2fInJMEVector2f(TexCoord2f textureCoordinate, Vector2f vector)
   {
      vector.setX(textureCoordinate.getX());
      vector.setY(textureCoordinate.getY());
   }

   public static float[] toPointCloudFloatArray(List<Point3d> points)
   {
      float[] ret = new float[points.size() * 3];
      for (int i = 0; i < points.size(); i++)
      {
         Point3d point3d = points.get(i);
         ret[i * 3 + 0] = (float) point3d.getX();
         ret[i * 3 + 1] = (float) point3d.getY();
         ret[i * 3 + 2] = (float) point3d.getZ();
      }

      return ret;
   }

   public static float[] toPointCloudFloatArrayInYUp(List<Point3d> points)
   {
      float[] ret = new float[points.size() * 3];
      for (int i = 0; i < points.size(); i++)
      {
         Point3d point3d = points.get(i);
         ret[i * 3 + 0] = (float) point3d.getY();
         ret[i * 3 + 1] = (float) point3d.getZ();
         ret[i * 3 + 2] = (float) point3d.getX();
      }

      return ret;
   }

   public static Transform j3dTransform3DToJMETransform(RigidBodyTransform transform3D)
   {
      Quat4f quat = new Quat4f();
      javax.vecmath.Vector3f vector = new javax.vecmath.Vector3f();
      transform3D.get(quat, vector);
      Vector3f jmeVector = new Vector3f(vector.getX(), vector.getY(), vector.getZ());
      Quaternion jmeQuat = new Quaternion(quat.getX(), quat.getY(), quat.getZ(), quat.getW());
      Transform ret = new Transform(jmeVector, jmeQuat, new Vector3f(1.0f, 1.0f, 1.0f));

      return ret;
   }

   public static RigidBodyTransform jmeTransformToTransform3D(Transform jmeTransform)
   {
      Quaternion jmeQuat = jmeTransform.getRotation();
      Vector3f jmeVect = jmeTransform.getTranslation();
      Quat4d quat = new Quat4d(jmeQuat.getX(), jmeQuat.getY(), jmeQuat.getZ(), jmeQuat.getW());
      Vector3d vect = new Vector3d(jmeVect.getX(), jmeVect.getY(), jmeVect.getZ());
      RigidBodyTransform ret = new RigidBodyTransform(quat, vect);

      return ret;
   }
}
