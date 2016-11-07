package us.ihmc.avatar.initialSetup;

import javax.vecmath.Vector3d;

public class SquaredUpDRCDemo01RobotAtPlatformsInitialSetup extends OffsetAndYawRobotInitialSetup
{
   private static final Vector3d firstPlatform = new Vector3d(-1.6, -3.2, 0.0);
   private static final Vector3d lastPlatform = new Vector3d(-6.5, -8.2, 0.0);
   
   private static final double offsetX = -1.8943, offsetY = -1.8745;
   private static final double yaw = Math.atan2(offsetY, offsetX);

   private SquaredUpDRCDemo01RobotAtPlatformsInitialSetup(Vector3d additionalOffset, double yaw)
   {
      super(additionalOffset, yaw);
   }
   
   public static SquaredUpDRCDemo01RobotAtPlatformsInitialSetup createInitialSetupAtNthPlatform(int nthPlatform)
   {
      double alpha = ((double) nthPlatform)/7.0;
      Vector3d startingLocation = morph(firstPlatform, lastPlatform, alpha);
      
      return new SquaredUpDRCDemo01RobotAtPlatformsInitialSetup(startingLocation, yaw);
   }

   public static SquaredUpDRCDemo01RobotAtPlatformsInitialSetup createInitialSetupAtNthWall(int nthWall)
   {
      double alpha = ((double) nthWall)/7.0;
      Vector3d startingLocation = morph(firstPlatform, lastPlatform, alpha);
      startingLocation.add(new Vector3d(-1.0 - 0.1, 1.0 - 0.1, 0.0));
      return new SquaredUpDRCDemo01RobotAtPlatformsInitialSetup(startingLocation, yaw - Math.PI/2.0);
   }
   
   public static SquaredUpDRCDemo01RobotAtPlatformsInitialSetup createInitialSetupAtNthPlatformToesTouching(int nthPlatform)
   {
      double alpha = ((double) nthPlatform)/7.0;
      Vector3d startingLocation = morph(firstPlatform, lastPlatform, alpha);
      
      startingLocation.add(new Vector3d(-0.11, -0.16, 0.0));
      
      return new SquaredUpDRCDemo01RobotAtPlatformsInitialSetup(startingLocation, yaw);
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

