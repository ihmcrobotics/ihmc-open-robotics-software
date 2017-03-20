package us.ihmc.modelFileLoaders;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFInertia;
import us.ihmc.robotics.dataStructures.MutableColor;

public class ModelFileLoaderConversionsHelper
{

   public static String sanitizeJointName(String dirtyName)
   {
      return dirtyName.trim().replaceAll("[//[//]///]", "").replace(".", "_");
   }

   public static Vector3D stringToNormalizedVector3d(String vector)
   {
      Vector3D vector3d = stringToVector3d(vector);
      vector3d.normalize();
      return vector3d;

   }
   
   public static Vector3D stringToVector3d(String vector)
   {
      String[] vecString = vector.split("\\s+");
      Vector3D vector3d = new Vector3D(Double.parseDouble(vecString[0]), Double.parseDouble(vecString[1]), Double.parseDouble(vecString[2]));
      return vector3d;
   }
   
   public static MutableColor stringToColor(String color)
   {
      String[] vecString = color.split("\\s+");
      MutableColor color3f = new MutableColor(Float.parseFloat(vecString[0]), Float.parseFloat(vecString[1]), Float.parseFloat(vecString[2]));
      return color3f;
   }


   
   public static Vector2D stringToVector2d(String xy)
   {
      String[] vecString = xy.split("\\s+");
   
      Vector2D vector = new Vector2D(Double.parseDouble(vecString[0]), Double.parseDouble(vecString[1]));
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
      Vector3D translationVector = new Vector3D();
      translationVector.setX(Double.parseDouble(data[0]));
      translationVector.setY(Double.parseDouble(data[1]));
      translationVector.setZ(Double.parseDouble(data[2]));
      translation.setTranslationAndIdentityRotation(translationVector);
   
      RigidBodyTransform rotation = new RigidBodyTransform();
      Vector3D eulerAngels = new Vector3D();
      eulerAngels.setX(Double.parseDouble(data[3]));
      eulerAngels.setY(Double.parseDouble(data[4]));
      eulerAngels.setZ(Double.parseDouble(data[5]));
      rotation.setRotationEulerAndZeroTranslation(eulerAngels);
   
      ret.set(translation);
      ret.multiply(rotation);
   
      return ret;
   }

   public static Matrix3D sdfInertiaToMatrix3d(SDFInertia sdfInertia)
   {
      
      Matrix3D inertia = new Matrix3D();
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
