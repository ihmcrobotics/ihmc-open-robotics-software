package us.ihmc.avatar.initialSetup;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class OffsetAndYawRobotInitialSetup
{
   private final Vector3D additionalOffset = new Vector3D();
   private double yaw;
   private double groundHeight;

   public OffsetAndYawRobotInitialSetup()
   {
      this(0.0, new Vector3D(), 0.0);
   }
   
   public OffsetAndYawRobotInitialSetup(double yaw)
   {
      this(0.0, new Vector3D(), yaw);
   }
   
   public OffsetAndYawRobotInitialSetup(double offsetX, double offsetY, double offsetZ, double yaw)
   {
      this(0.0, new Vector3D(offsetX, offsetY, offsetZ), yaw);
   }

   public OffsetAndYawRobotInitialSetup(double offsetX, double offsetY, double offsetZ)
   {
      this(0.0, new Vector3D(offsetX, offsetY, offsetZ), Math.atan2(offsetY, offsetX));
   }

   public OffsetAndYawRobotInitialSetup(Tuple3DReadOnly additionalOffset, double yaw)
   {
      this(0.0, additionalOffset, yaw);
   }

   public OffsetAndYawRobotInitialSetup(double groundHeight, Tuple3DReadOnly additionalOffset, double yaw)
   {
      this.groundHeight = groundHeight;
      this.additionalOffset.set(additionalOffset);
      this.yaw = yaw;
   }
   
   public Vector3D getAdditionalOffset()
   {
      return additionalOffset;
   }
   
   public double getGroundHeight()
   {
      return groundHeight;
   }
   
   public double getYaw()
   {
      return yaw;
   }

   public void addAdditionalOffset(Vector3DReadOnly additionalOffset)
   {
      this.additionalOffset.add(additionalOffset);
   }

   public void addAdditionalYaw(double additionalYaw)
   {
      this.yaw += additionalYaw;
   }

   public void addAdditionalOffsetAndYaw(Vector3DReadOnly additionalOffset, double additionalYaw)
   {
      addAdditionalOffset(additionalOffset);
      addAdditionalYaw(additionalYaw);
   }

}
