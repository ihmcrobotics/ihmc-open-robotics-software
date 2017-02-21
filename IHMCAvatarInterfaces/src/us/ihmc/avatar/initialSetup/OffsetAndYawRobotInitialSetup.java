package us.ihmc.avatar.initialSetup;

import us.ihmc.euclid.tuple3D.Vector3D;

public class OffsetAndYawRobotInitialSetup
{
   private final Vector3D additionalOffset = new Vector3D();
   private final double yaw;
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

   public OffsetAndYawRobotInitialSetup(Vector3D additionalOffset, double yaw)
   {
      this(0.0, additionalOffset, yaw);
   }

   public OffsetAndYawRobotInitialSetup(double groundHeight, Vector3D additionalOffset, double yaw)
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
}
