package us.ihmc.darpaRoboticsChallenge;

import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.initialSetup.OffsetAndYawRobotInitialSetup;

public class DRCOffsetStartingLocation implements DRCStartingLocation
{
   private OffsetAndYawRobotInitialSetup startingLocationOffset;

   public DRCOffsetStartingLocation(double offsetX, double offsetY, double offsetZ, double yaw)
   {
      startingLocationOffset = new OffsetAndYawRobotInitialSetup(offsetX, offsetY, offsetZ, yaw);
   }

   public DRCOffsetStartingLocation(Vector3d positionOffset, double yaw)
   {
      startingLocationOffset = new OffsetAndYawRobotInitialSetup(positionOffset, yaw);
   }

   @Override
   public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
   {
      return startingLocationOffset;
   }

}
