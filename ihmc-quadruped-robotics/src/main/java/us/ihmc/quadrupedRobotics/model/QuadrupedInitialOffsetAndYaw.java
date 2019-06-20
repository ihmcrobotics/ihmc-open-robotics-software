package us.ihmc.quadrupedRobotics.model;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class QuadrupedInitialOffsetAndYaw
{
   private final Vector3D additionalOffset = new Vector3D();
   private final double yaw;

   public QuadrupedInitialOffsetAndYaw()
   {
      this(new Vector3D(), 0.0);
   }

   public QuadrupedInitialOffsetAndYaw(double yaw)
   {
      this(new Vector3D(), yaw);
   }

   public QuadrupedInitialOffsetAndYaw(double offsetX, double offsetY, double offsetZ, double yaw)
   {
      this(new Vector3D(offsetX, offsetY, offsetZ), yaw);
   }

   public QuadrupedInitialOffsetAndYaw(double offsetX, double offsetY, double offsetZ)
   {
      this(new Vector3D(offsetX, offsetY, offsetZ), Math.atan2(offsetY, offsetX));
   }

   public QuadrupedInitialOffsetAndYaw(Tuple3DReadOnly additionalOffset)
   {
      this(additionalOffset, 0.0);
   }

   public QuadrupedInitialOffsetAndYaw(Tuple3DReadOnly additionalOffset, double yaw)
   {
      this.additionalOffset.set(additionalOffset);
      this.yaw = yaw;
   }

   public Vector3D getAdditionalOffset()
   {
      return additionalOffset;
   }

   public double getYaw()
   {
      return yaw;
   }
}
