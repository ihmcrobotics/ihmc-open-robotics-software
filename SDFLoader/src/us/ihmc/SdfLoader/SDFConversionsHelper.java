package us.ihmc.SdfLoader;

import java.util.Arrays;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.xmlDescription.SDFInertia;

public class SDFConversionsHelper
{

   public static String sanitizeJointName(String dirtyName)
   {
      return dirtyName.replace(".", "_");
   }

   public static Vector3d stringToAxis(String xyz)
   
   {
      String[] vecString = xyz.split("\\s");
   
      Vector3d axis = new Vector3d(Double.parseDouble(vecString[0]), Double.parseDouble(vecString[1]), Double.parseDouble(vecString[2]));
      axis.normalize();
   
      return axis;
   
   }

   public static Transform3D poseToTransform(String pose)
   {
      Transform3D ret = new Transform3D();
      if(pose == null)
      {
         return ret;
      }
      pose = pose.trim();
      String[] data = pose.split("\\s+");
      
      Transform3D translation = new Transform3D();
      Vector3d translationVector = new Vector3d();
      translationVector.setX(Double.parseDouble(data[0]));
      translationVector.setY(Double.parseDouble(data[1]));
      translationVector.setZ(Double.parseDouble(data[2]));
      translation.set(translationVector);
   
      Transform3D rotation = new Transform3D();
      Vector3d eulerAngels = new Vector3d();
      eulerAngels.setX(Double.parseDouble(data[3]));
      eulerAngels.setY(Double.parseDouble(data[4]));
      eulerAngels.setZ(Double.parseDouble(data[5]));
      rotation.setEuler(eulerAngels);
   
      ret.mul(translation, rotation);
   
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
         inertia.m00 = ixx;
         inertia.m01 = ixy;
         inertia.m02 = ixz;
         inertia.m10 = ixy;
         inertia.m11 = iyy;
         inertia.m12 = iyz;
         inertia.m20 = ixz;
         inertia.m21 = iyz;
         inertia.m22 = izz;
      }
      return inertia;
   }
}
