package us.ihmc.perception.tools;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Vector4D;

public class PerceptionEuclidTools
{
   public static float[] toArray(Pose3D pose)
   {
      return new float[] {(float) pose.getYaw(),
                          (float) pose.getPitch(),
                          (float) pose.getRoll(),
                          (float) pose.getX(),
                          (float) pose.getY(),
                          (float) pose.getZ()};
   }

   public static float[] toArray(Vector4D plane)
   {
      return new float[] {plane.getX32(), plane.getY32(), plane.getZ32(), plane.getS32()};
   }

   public static double[] toArray(RigidBodyTransform transform)
   {
      double[] transformArray = new double[16];
      transform.get(transformArray);
      return transformArray;
   }

   public static float[] toFloatArray(RigidBodyTransform transform)
   {
      float[] transformArray = new float[16];
      transform.get(transformArray);
      return transformArray;
   }
}
