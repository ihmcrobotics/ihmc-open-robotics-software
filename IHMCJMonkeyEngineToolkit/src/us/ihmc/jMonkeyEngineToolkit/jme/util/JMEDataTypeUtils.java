package us.ihmc.jMonkeyEngineToolkit.jme.util;

import java.awt.Color;
import java.util.List;

import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Transform;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.robotics.dataStructures.MutableColor;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.Ray3d;

public class JMEDataTypeUtils
{
   public static Point3D jmeVector3fToJ3DPoint3d(Vector3f original)
   {
      return new Point3D(original.getX(), original.getY(), original.getZ());
   }

   public static Vector3D jmeVector3fToVecmathVector3d(Vector3f original)
   {
      return new Vector3D(original.getX(), original.getY(), original.getZ());
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

   public static Vector3f vecMathTuple3dToJMEVector3f(Tuple3DBasics original)
   {
      Vector3f target = new Vector3f();
      packVecMathTuple3dInJMEVector3f(original, target);

      return target;
   }

   public static Quaternion vecMathQuat4dToJMEQuaternion(QuaternionReadOnly original)
   {
      Quaternion target = new Quaternion();
      packVectMathQuat4dInJMEQuaternion(original, target);

      return target;
   }

   public static us.ihmc.euclid.tuple4D.Quaternion jMEQuaternionToVecMathQuat4d(Quaternion original)
   {
      us.ihmc.euclid.tuple4D.Quaternion target = new us.ihmc.euclid.tuple4D.Quaternion();
      packJMEQuaterionInVecMathQuat4d(original, target);

      return target;
   }

   public static void packJMEVector3fInVecMathTuple3d(Vector3f original, Tuple3DBasics target)
   {
      target.set(original.getX(), original.getY(), original.getZ());
   }

   public static void packJMEQuaterionInVecMathQuat4d(Quaternion original, QuaternionBasics target)
   {
      target.set(original.getX(), original.getY(), original.getZ(), original.getW());

      // do not remove the normalization. 
      // The conversion from float to double generates very tiny differences which make the 
      // quaternion SLIGHTLY not normal.

      target.normalize();
   }

   public static void packVecMathTuple3dInJMEVector3f(Tuple3DBasics original, Vector3f target)
   {
      target.set((float) original.getX(), (float) original.getY(), (float) original.getZ());
   }

   public static void packVecMathTuple3fInJMEVector3f(Tuple3DBasics original, Vector3f target)
   {
      target.set(original.getX32(), original.getY32(), original.getZ32());
   }

   public static void packVectMathQuat4dInJMEQuaternion(QuaternionReadOnly original, Quaternion target)
   {
      target.set((float) original.getX(), (float) original.getY(), (float) original.getZ(), (float) original.getS());
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
      us.ihmc.euclid.tuple4D.Quaternion quat4d = new us.ihmc.euclid.tuple4D.Quaternion();
      packVectMathQuat4dInJMEQuaternion(original.getQuaternion(), target);
   }

   public static void packFramePoseInJMEQuaternion(FramePose original, Quaternion target)
   {
      us.ihmc.euclid.tuple4D.Quaternion quat4d = new us.ihmc.euclid.tuple4D.Quaternion();
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

   public static ColorRGBA jMEColorRGBAFromVecMathColor3f(MutableColor originalColor, double alpha)
   {
      float r = originalColor.getX();
      float g = originalColor.getY();
      float b = originalColor.getZ();

      return new ColorRGBA(r, g, b, (float) alpha);
   }

   public static Vector3f[] vecMathTuple3fArrayToJMEVector3fArray(Tuple3DBasics[] original)
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
      vector.setX(textureCoordinate.getX32());
      vector.setY(textureCoordinate.getY32());
   }

   public static float[] toPointCloudFloatArray(List<Point3D> points)
   {
      float[] ret = new float[points.size() * 3];
      for (int i = 0; i < points.size(); i++)
      {
         Point3D point3d = points.get(i);
         ret[i * 3 + 0] = (float) point3d.getX();
         ret[i * 3 + 1] = (float) point3d.getY();
         ret[i * 3 + 2] = (float) point3d.getZ();
      }

      return ret;
   }

   public static float[] toPointCloudFloatArrayInYUp(List<Point3D> points)
   {
      float[] ret = new float[points.size() * 3];
      for (int i = 0; i < points.size(); i++)
      {
         Point3D point3d = points.get(i);
         ret[i * 3 + 0] = (float) point3d.getY();
         ret[i * 3 + 1] = (float) point3d.getZ();
         ret[i * 3 + 2] = (float) point3d.getX();
      }

      return ret;
   }

   public static Transform j3dTransform3DToJMETransform(RigidBodyTransform transform3D)
   {
      Quaternion32 quat = new Quaternion32();
      us.ihmc.euclid.tuple3D.Vector3D32 vector = new us.ihmc.euclid.tuple3D.Vector3D32();
      transform3D.get(quat, vector);
      Vector3f jmeVector = new Vector3f(vector.getX32(), vector.getY32(), vector.getZ32());
      Quaternion jmeQuat = new Quaternion(quat.getX32(), quat.getY32(), quat.getZ32(), quat.getS32());
      Transform ret = new Transform(jmeVector, jmeQuat, new Vector3f(1.0f, 1.0f, 1.0f));

      return ret;
   }

   public static RigidBodyTransform jmeTransformToTransform3D(Transform jmeTransform)
   {
      Quaternion jmeQuat = jmeTransform.getRotation();
      Vector3f jmeVect = jmeTransform.getTranslation();
      us.ihmc.euclid.tuple4D.Quaternion quat = new us.ihmc.euclid.tuple4D.Quaternion(jmeQuat.getX(), jmeQuat.getY(), jmeQuat.getZ(), jmeQuat.getW());
      Vector3D vect = new Vector3D(jmeVect.getX(), jmeVect.getY(), jmeVect.getZ());
      RigidBodyTransform ret = new RigidBodyTransform(quat, vect);

      return ret;
   }
}
