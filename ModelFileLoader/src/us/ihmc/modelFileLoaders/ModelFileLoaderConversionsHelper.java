package us.ihmc.modelFileLoaders;

import javax.vecmath.Color3f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFInertia;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class ModelFileLoaderConversionsHelper
{

   public static String sanitizeJointName(String dirtyName)
   {
      return dirtyName.trim().replaceAll("[//[//]///]", "").replace(".", "_");
   }

   public static Vector3d stringToNormalizedVector3d(String vector)
   {
      Vector3d vector3d = stringToVector3d(vector);
      vector3d.normalize();
      return vector3d;

   }
   
   public static Vector3d stringToVector3d(String vector)
   {
      String[] vecString = vector.split("\\s+");
      Vector3d vector3d = new Vector3d(Double.parseDouble(vecString[0]), Double.parseDouble(vecString[1]), Double.parseDouble(vecString[2]));
      return vector3d;
   }
   
   public static Color3f stringToColor(String color)
   {
      String[] vecString = color.split("\\s+");
      Color3f color3f = new Color3f(Float.parseFloat(vecString[0]), Float.parseFloat(vecString[1]), Float.parseFloat(vecString[2]));
      return color3f;
   }


   
   public static Vector2d stringToVector2d(String xy)
   {
      String[] vecString = xy.split("\\s+");
   
      Vector2d vector = new Vector2d(Double.parseDouble(vecString[0]), Double.parseDouble(vecString[1]));
      return vector;
   
   }

   public static RigidBodyTransform poseToTransform(String pose)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      if(pose == null)
      {
         return ret;
      }
      pose = pose.trim();
      String[] data = pose.split("\\s+");
      
      RigidBodyTransform translation = new RigidBodyTransform();
      Vector3d translationVector = new Vector3d();
      translationVector.setX(Double.parseDouble(data[0]));
      translationVector.setY(Double.parseDouble(data[1]));
      translationVector.setZ(Double.parseDouble(data[2]));
      translation.setTranslationAndIdentityRotation(translationVector);
   
      RigidBodyTransform rotation = new RigidBodyTransform();
      Vector3d eulerAngels = new Vector3d();
      eulerAngels.setX(Double.parseDouble(data[3]));
      eulerAngels.setY(Double.parseDouble(data[4]));
      eulerAngels.setZ(Double.parseDouble(data[5]));
      rotation.setRotationEulerAndZeroTranslation(eulerAngels);
   
      ret.multiply(translation, rotation);
   
      return ret;
   }

   public static Matrix3d sdfInertiaToMatrix3d(SDFInertia sdfInertia)
   {
      
      Matrix3d inertia = new Matrix3d();
      if(sdfInertia != null)
      {
         double ixx = Double.parseDouble(sdfInertia.getIxx());
         double ixy = Double.parseDouble(sdfInertia.getIxy());
         double ixz = Double.parseDouble(sdfInertia.getIxz());
         double iyy = Double.parseDouble(sdfInertia.getIyy());
         double iyz = Double.parseDouble(sdfInertia.getIyz());
         double izz = Double.parseDouble(sdfInertia.getIzz());
         inertia.setM00(ixx);
         inertia.setM01(ixy);
         inertia.setM02(ixz);
         inertia.setM10(ixy);
         inertia.setM11(iyy);
         inertia.setM12(iyz);
         inertia.setM20(ixz);
         inertia.setM21(iyz);
         inertia.setM22(izz);
      }
      return inertia;
   }

}
