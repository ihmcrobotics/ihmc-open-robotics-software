package us.ihmc.avatar.initialSetup;

import javax.vecmath.Vector3d;

public class OffsetAndYawRobotInitialSetup
{
   private final Vector3d additionalOffset = new Vector3d();
   private final double yaw;
   private double groundHeight;

   public OffsetAndYawRobotInitialSetup()
   {
      this(0.0, new Vector3d(), 0.0);
   }
   
   public OffsetAndYawRobotInitialSetup(double yaw)
   {
      this(0.0, new Vector3d(), yaw);
   }
   
   public OffsetAndYawRobotInitialSetup(double offsetX, double offsetY, double offsetZ, double yaw)
   {
      this(0.0, new Vector3d(offsetX, offsetY, offsetZ), yaw);
   }

   public OffsetAndYawRobotInitialSetup(double offsetX, double offsetY, double offsetZ)
   {
      this(0.0, new Vector3d(offsetX, offsetY, offsetZ), Math.atan2(offsetY, offsetX));
   }

   public OffsetAndYawRobotInitialSetup(Vector3d additionalOffset, double yaw)
   {
      this(0.0, additionalOffset, yaw);
   }

   public OffsetAndYawRobotInitialSetup(double groundHeight, Vector3d additionalOffset, double yaw)
   {
      this.groundHeight = groundHeight;
      this.additionalOffset.set(additionalOffset);
      this.yaw = yaw;
   }
   
   public Vector3d getAdditionalOffset()
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
