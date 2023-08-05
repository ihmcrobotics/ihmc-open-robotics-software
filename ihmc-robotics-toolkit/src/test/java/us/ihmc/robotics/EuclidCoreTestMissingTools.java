package us.ihmc.robotics;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class EuclidCoreTestMissingTools
{
   public static String toStringFullPrecision(RigidBodyTransform rigidBodyTransform)
   {
      return EuclidCoreIOTools.getRigidBodyTransformString(EuclidCoreIOTools.getStringFormat(18, 18), rigidBodyTransform);
   }
}
