package us.ihmc.avatar;

import java.util.HashMap;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.initialSetup.SquaredUpDRCDemo01Robot;

public enum  RoughTerrainEnvironmentStartingLocation implements DRCStartingLocation
{
   DEFAULT,
   IN_FRONT_OF_STAIRS_1,
   IN_FRONT_OF_STAIRS_2,
   IN_FRONT_OF_STAIRS_3,
   IN_FRONT_OF_ROUNDED_ROCKS_1,
   IN_FRONT_OF_ROUNDED_ROCKS_2,
   IN_FRONT_OF_FLAT_ROCKS;
   
   public static final HashMap<String, OffsetAndYawRobotInitialSetup> initialSetupMap = new HashMap<String, OffsetAndYawRobotInitialSetup>();
   
   static
   {
      addMapping(RoughTerrainEnvironmentStartingLocation.DEFAULT, new SquaredUpDRCDemo01Robot(0.0, 0.0, 0.0, 0));
      addMapping(RoughTerrainEnvironmentStartingLocation.IN_FRONT_OF_STAIRS_1, new SquaredUpDRCDemo01Robot(0.0, 4.5, Math.toRadians(45.0), 0));
      addMapping(RoughTerrainEnvironmentStartingLocation.IN_FRONT_OF_STAIRS_2, new SquaredUpDRCDemo01Robot(0.0, 3.5, 0.0, 0));
      addMapping(RoughTerrainEnvironmentStartingLocation.IN_FRONT_OF_STAIRS_3, new SquaredUpDRCDemo01Robot(0.0, 4.5, Math.toRadians(-45.0), 0));
      addMapping(RoughTerrainEnvironmentStartingLocation.IN_FRONT_OF_ROUNDED_ROCKS_1, new SquaredUpDRCDemo01Robot(0.0, 4.0, Math.toRadians(90.0), 0));
      addMapping(RoughTerrainEnvironmentStartingLocation.IN_FRONT_OF_ROUNDED_ROCKS_2, new SquaredUpDRCDemo01Robot(0.0, 3.2, Math.toRadians(135.0), 0));
      addMapping(RoughTerrainEnvironmentStartingLocation.IN_FRONT_OF_FLAT_ROCKS, new SquaredUpDRCDemo01Robot(0.0, 2.8, Math.toRadians(180.0), 0));
   }
   
   private static void addMapping(RoughTerrainEnvironmentStartingLocation drcDemo01StartingLocation, OffsetAndYawRobotInitialSetup robotInitialSetup)
   {
      initialSetupMap.put(drcDemo01StartingLocation.toString(), robotInitialSetup);
   }
   
   public static String optionsToString()
   {
      String ret = "[ ";
      for (RoughTerrainEnvironmentStartingLocation location : RoughTerrainEnvironmentStartingLocation.values())
      {
         ret = ret + location.toString() + ", ";
      }
      ret = ret + "]";
      return ret;
   }

   @Override
   public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
   {
      OffsetAndYawRobotInitialSetup startingLocation = initialSetupMap.get(this.toString());
      return startingLocation;
   }
}
