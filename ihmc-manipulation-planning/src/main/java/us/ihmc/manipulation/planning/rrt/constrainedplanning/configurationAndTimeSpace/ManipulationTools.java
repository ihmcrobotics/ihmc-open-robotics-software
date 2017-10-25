package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class ManipulationTools
{

   /**
    * quat1 * displacementQuat = quat2
    */
   public static final void computeDisplacementQuaternion(Quaternion quat1, Quaternion quat2, Quaternion displacementQuat)
   {
      quat1.inverse();
      displacementQuat.multiply(quat1, quat2);
   }

   /**
    * rotationAngle from quat1 to quat2
    */
   public static final void computeDisplacementQuaternion(Quaternion quat1, Quaternion quat2, double rotationAngle)
   {
      Quaternion displacementQuat = new Quaternion();
      computeDisplacementQuaternion(quat1, quat2, displacementQuat);

      Vector3D rotationVectorToPack = new Vector3D();
      RotationVectorConversion.convertQuaternionToRotationVector(displacementQuat, rotationVectorToPack);

      rotationAngle = Math.sqrt(rotationVectorToPack.getX() * rotationVectorToPack.getX() + rotationVectorToPack.getY() * rotationVectorToPack.getY()
            + rotationVectorToPack.getZ() * rotationVectorToPack.getZ());
   }
}
