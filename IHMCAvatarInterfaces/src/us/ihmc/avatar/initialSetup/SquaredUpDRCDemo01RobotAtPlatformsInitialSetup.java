package us.ihmc.avatar.initialSetup;

import us.ihmc.euclid.tuple3D.Vector3D;

public class SquaredUpDRCDemo01RobotAtPlatformsInitialSetup extends OffsetAndYawRobotInitialSetup
{
   private static final Vector3D firstPlatform = new Vector3D(-1.6, -3.2, 0.0);
   private static final Vector3D lastPlatform = new Vector3D(-6.5, -8.2, 0.0);
   
   private static final double offsetX = -1.8943, offsetY = -1.8745;
   private static final double yaw = Math.atan2(offsetY, offsetX);

   private SquaredUpDRCDemo01RobotAtPlatformsInitialSetup(Vector3D additionalOffset, double yaw)
   {
      super(additionalOffset, yaw);
   }
   
   public static SquaredUpDRCDemo01RobotAtPlatformsInitialSetup createInitialSetupAtNthPlatform(int nthPlatform)
   {
      double alpha = ((double) nthPlatform)/7.0;
      Vector3D startingLocation = morph(firstPlatform, lastPlatform, alpha);
      
      return new SquaredUpDRCDemo01RobotAtPlatformsInitialSetup(startingLocation, yaw);
   }

   public static SquaredUpDRCDemo01RobotAtPlatformsInitialSetup createInitialSetupAtNthWall(int nthWall)
   {
      double alpha = ((double) nthWall)/7.0;
      Vector3D startingLocation = morph(firstPlatform, lastPlatform, alpha);
      startingLocation.add(new Vector3D(-1.0 - 0.1, 1.0 - 0.1, 0.0));
      return new SquaredUpDRCDemo01RobotAtPlatformsInitialSetup(startingLocation, yaw - Math.PI/2.0);
   }
   
   public static SquaredUpDRCDemo01RobotAtPlatformsInitialSetup createInitialSetupAtNthPlatformToesTouching(int nthPlatform)
   {
      double alpha = ((double) nthPlatform)/7.0;
      Vector3D startingLocation = morph(firstPlatform, lastPlatform, alpha);
      
      startingLocation.add(new Vector3D(-0.11, -0.16, 0.0));
      
      return new SquaredUpDRCDemo01RobotAtPlatformsInitialSetup(startingLocation, yaw);
   }
   
   public static Vector3D morph(Vector3D point1, Vector3D point2, double alpha)
   {
      Vector3D framePoint1 = new Vector3D(point1);
      Vector3D framePoint2 = new Vector3D(point2);

      framePoint1.scale(1.0 - alpha);
      framePoint2.scale(alpha);
      framePoint1.add(framePoint2);


      return framePoint1;
   }

}

