package us.ihmc.avatar.initialSetup;

import javax.vecmath.Vector3d;

public class SquaredUpDRCDemo01RobotOnPlatformsInitialSetup extends OffsetAndYawRobotInitialSetup
{
   private static final Vector3d firstPlatform = new Vector3d(-2.0, -3.6, 0.05);
   private static final Vector3d lastPlatform = new Vector3d(-6.9, -8.6, 0.4);
   
   private static final double offsetX = -1.8943, offsetY = -1.8745;
   private static final double yaw = Math.atan2(offsetY, offsetX);
   
   public SquaredUpDRCDemo01RobotOnPlatformsInitialSetup(Vector3d additionalOffset, double yaw)
   {
      super(additionalOffset, yaw);
   }

   public static SquaredUpDRCDemo01RobotOnPlatformsInitialSetup createInitialSetupOnNthPlatform(int nthPlatform)
   {
      double alpha = ((double) nthPlatform)/7.0;
      Vector3d startingLocation = morph(firstPlatform, lastPlatform, alpha);
      
      return new SquaredUpDRCDemo01RobotOnPlatformsInitialSetup(startingLocation, yaw);
   }
   
   public static Vector3d morph(Vector3d point1, Vector3d point2, double alpha)
   {
      Vector3d framePoint1 = new Vector3d(point1);
      Vector3d framePoint2 = new Vector3d(point2);

      framePoint1.scale(1.0 - alpha);
      framePoint2.scale(alpha);
      framePoint1.add(framePoint2);


      return framePoint1;
   }
   
}
