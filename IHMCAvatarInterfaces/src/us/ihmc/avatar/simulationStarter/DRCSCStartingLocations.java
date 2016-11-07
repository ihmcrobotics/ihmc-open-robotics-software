package us.ihmc.avatar.simulationStarter;

import java.util.HashMap;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;

public enum DRCSCStartingLocations implements DRCStartingLocation
{
   DEFAULT,
   DEBRIS_START,
   DEBRIS_END,
   DRILL,
   PLUG,
   WALKING_START,
   WALKING_END,
   STAIRS_START,
   STAIRS_END,
   SUPRISE;

   public static final HashMap<String, OffsetAndYawRobotInitialSetup> initialSetupMap = new HashMap<String, OffsetAndYawRobotInitialSetup>();

   static
   {
      addMapping(DRCSCStartingLocations.DEFAULT, new OffsetAndYawRobotInitialSetup(0,2,0,Math.toRadians(-90.0)));
      addMapping(DRCSCStartingLocations.DEBRIS_START, new OffsetAndYawRobotInitialSetup(0.5,-1.0,0,Math.toRadians(-90.0)));
      addMapping(DRCSCStartingLocations.DEBRIS_END, new OffsetAndYawRobotInitialSetup(0.5,-3.5, 0,Math.toRadians(-90.0)));
      addMapping(DRCSCStartingLocations.DRILL, new OffsetAndYawRobotInitialSetup(0,-6.5, 0,Math.toRadians(-180.0)));
      addMapping(DRCSCStartingLocations.PLUG, new OffsetAndYawRobotInitialSetup(0,-8.5, 0,Math.toRadians(-180.0)));
      addMapping(DRCSCStartingLocations.WALKING_START, new OffsetAndYawRobotInitialSetup(0,-8.5, 0,Math.toRadians(-90.0)));
      addMapping(DRCSCStartingLocations.WALKING_END, new OffsetAndYawRobotInitialSetup(0.35,-13.5, 0,Math.toRadians(-90.0)));
      addMapping(DRCSCStartingLocations.STAIRS_START, new OffsetAndYawRobotInitialSetup(1.0,-13.5, 0,Math.toRadians(-90.0)));
      addMapping(DRCSCStartingLocations.STAIRS_END, new OffsetAndYawRobotInitialSetup(1.0,-16, 0.9144,Math.toRadians(-90.0)));
   };

   private static void addMapping(DRCSCStartingLocations drcDemo01StartingLocation, OffsetAndYawRobotInitialSetup robotInitialSetup)
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
      for (DRCSCStartingLocations location : DRCSCStartingLocations.values())
      {
         ret = ret + location.toString() + ", ";
      }
      ret = ret + "]";
      return ret;
   }
}
