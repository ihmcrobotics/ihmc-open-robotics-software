package us.ihmc.avatar.initialSetup;

import us.ihmc.euclid.tuple3D.Vector3D;

public class SquaredUpDRCDemo01RobotOnPlatformsInitialSetup extends OffsetAndYawRobotInitialSetup
{
   private static final Vector3D firstPlatform = new Vector3D(-2.0, -3.6, 0.05);
   private static final Vector3D lastPlatform = new Vector3D(-6.9, -8.6, 0.4);
   
   private static final double offsetX = -1.8943, offsetY = -1.8745;
   private static final double yaw = Math.atan2(offsetY, offsetX);
   
   public SquaredUpDRCDemo01RobotOnPlatformsInitialSetup(Vector3D additionalOffset, double yaw)
   {
      super(additionalOffset, yaw);
   }

   public static SquaredUpDRCDemo01RobotOnPlatformsInitialSetup createInitialSetupOnNthPlatform(int nthPlatform)
   {
      double alpha = ((double) nthPlatform)/7.0;
      Vector3D startingLocation = morph(firstPlatform, lastPlatform, alpha);
      
      return new SquaredUpDRCDemo01RobotOnPlatformsInitialSetup(startingLocation, yaw);
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
