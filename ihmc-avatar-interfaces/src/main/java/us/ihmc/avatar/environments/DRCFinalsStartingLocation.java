package us.ihmc.avatar.environments;

import java.util.HashMap;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;

public enum DRCFinalsStartingLocation implements DRCStartingLocation
{
   DEFAULT;

   public static final HashMap<String, OffsetAndYawRobotInitialSetup> initialSetupMap = new HashMap<String, OffsetAndYawRobotInitialSetup>();

   static
   {
      addMapping(DRCFinalsStartingLocation.DEFAULT, new OffsetAndYawRobotInitialSetup(0.4, 1.3, 0.0, -Math.PI / 2));
   }

   private static void addMapping(DRCFinalsStartingLocation drcDemo01StartingLocation, OffsetAndYawRobotInitialSetup robotInitialSetup)
   {
      initialSetupMap.put(drcDemo01StartingLocation.toString(), robotInitialSetup);
   }

   @Override
   public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
   {
      OffsetAndYawRobotInitialSetup startingLocation = initialSetupMap.get(this.toString());
      return startingLocation;
   }
   
   public static String optionsToString()
   {
      String ret = "[ ";
      for (DRCFinalsStartingLocation location : DRCFinalsStartingLocation.values())
      {
         ret = ret + location.toString() + ", ";
      }
      ret = ret + "]";
      return ret;
   }
   
}
