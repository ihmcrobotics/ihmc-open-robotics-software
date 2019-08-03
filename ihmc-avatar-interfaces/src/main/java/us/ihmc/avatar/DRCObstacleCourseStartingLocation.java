package us.ihmc.avatar;

import java.util.HashMap;

import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.initialSetup.SquaredUpDRCDemo01Robot;
import us.ihmc.avatar.initialSetup.SquaredUpDRCDemo01RobotAtPlatformsInitialSetup;
import us.ihmc.avatar.initialSetup.SquaredUpDRCDemo01RobotOnPlatformsInitialSetup;

public enum DRCObstacleCourseStartingLocation implements DRCStartingLocation
{
   DEFAULT,
   DRC_TRIALS_TRAINING_WALKING,
   DRC_TRIALS_QUALS,
   MID_LADDER,
   RAMP_BOTTOM,
   RAMP_TOP,
   NARROW_DOORWAY,
   BARRIERS,
   BARRIERS_FURTHER_IN,
   SMALL_PLATFORM,
   MEDIUM_PLATFORM,
   ON_MEDIUM_PLATFORM,
   EASY_STEPPING_STONES,
   STEPPING_STONES,
   STAIRS,
   TOP_OF_STAIRS,
   ROCKS,
   LADDER,
   IN_FRONT_OF_ZIGZAG_BLOCKS,
   IN_FRONT_OF_SECOND_ZIGZAG_BLOCKS,
   SINGLE_CYLINDERBLOCKS,
   TOP_OF_SLOPES,
   DEFAULT_BUT_ALMOST_PI,
   IN_FRONT_OF_CINDERBLOCK_FIELD,
   IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS,
   IN_FRONT_OF_CYLINDER_BLOCKS,
   IN_FRONT_OF_SLANTED_CINDERBLOCK_FIELD,
   OFFSET_ONE_METER_X_AND_Y,
   OFFSET_ONE_METER_X_AND_Y_ROTATED_PI,
   SMALL_PLATFORM_TURNED, SMALL_WALL;

   public static final HashMap<String, OffsetAndYawRobotInitialSetup> initialSetupMap = new HashMap<String, OffsetAndYawRobotInitialSetup>();

   static
   {
      addMapping(DRCObstacleCourseStartingLocation.BARRIERS, new SquaredUpDRCDemo01Robot(0.0, 1.88, Math.toRadians(-135.0), 0));
      addMapping(DRCObstacleCourseStartingLocation.BARRIERS_FURTHER_IN, new SquaredUpDRCDemo01Robot(0.0, 4.5, Math.toRadians(-135.0), 0));
      addMapping(DRCObstacleCourseStartingLocation.DEFAULT, new OffsetAndYawRobotInitialSetup());
      addMapping(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y, new SquaredUpDRCDemo01Robot(1.0, 1.0));
      addMapping(DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y_ROTATED_PI, new OffsetAndYawRobotInitialSetup(1.0, 1.0, 0, Math.PI));
      addMapping(DRCObstacleCourseStartingLocation.EASY_STEPPING_STONES, new OffsetAndYawRobotInitialSetup(-7.22, -0.8, 0.3, Math.PI - 0.01));
      addMapping(DRCObstacleCourseStartingLocation.MEDIUM_PLATFORM, SquaredUpDRCDemo01RobotAtPlatformsInitialSetup.createInitialSetupAtNthPlatformToesTouching(3));
      addMapping(DRCObstacleCourseStartingLocation.DRC_TRIALS_QUALS, new SquaredUpDRCDemo01Robot(0.0, 1.5, Math.toRadians(-41.1147), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.NARROW_DOORWAY, new SquaredUpDRCDemo01Robot(0.0, 2.7094, Math.toRadians(-89.2852), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.ON_MEDIUM_PLATFORM, SquaredUpDRCDemo01RobotOnPlatformsInitialSetup.createInitialSetupOnNthPlatform(3));
      addMapping(DRCObstacleCourseStartingLocation.RAMP_BOTTOM, new SquaredUpDRCDemo01Robot(0.0, 2.6179, 0.0, 0.0));
      addMapping(DRCObstacleCourseStartingLocation.RAMP_TOP, new SquaredUpDRCDemo01Robot(0.6255, 7.25, 0.0, 0.99*Math.PI));
      addMapping(DRCObstacleCourseStartingLocation.ROCKS, new OffsetAndYawRobotInitialSetup(0.0, 2.044, 0.0));
      addMapping(DRCObstacleCourseStartingLocation.DRC_TRIALS_TRAINING_WALKING, new SquaredUpDRCDemo01Robot(0.0, 3.0, Math.toRadians(45.0), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.IN_FRONT_OF_ZIGZAG_BLOCKS, new SquaredUpDRCDemo01Robot(0.0, 5.9, Math.toRadians(44.2), Math.toRadians(45.0)));
      addMapping(DRCObstacleCourseStartingLocation.IN_FRONT_OF_SECOND_ZIGZAG_BLOCKS, new SquaredUpDRCDemo01Robot(0.0, 21.2, Math.toRadians(44.2), Math.toRadians(180.0-45.0)));
      addMapping(DRCObstacleCourseStartingLocation.IN_FRONT_OF_CINDERBLOCK_FIELD, new SquaredUpDRCDemo01Robot(0.0, 7.0, Math.toRadians(44.2), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.IN_FRONT_OF_SLANTED_CINDERBLOCK_FIELD, new SquaredUpDRCDemo01Robot(0.0, 10.80, Math.toRadians(44.0), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS, new SquaredUpDRCDemo01Robot(0.0, 18.6, Math.toRadians(44.2), Math.toRadians(45.0)));
      addMapping(DRCObstacleCourseStartingLocation.IN_FRONT_OF_CYLINDER_BLOCKS, new SquaredUpDRCDemo01Robot(0.0, 10.1, Math.toRadians(44.5), Math.toRadians(45.0)));
      addMapping(DRCObstacleCourseStartingLocation.SMALL_PLATFORM, SquaredUpDRCDemo01RobotAtPlatformsInitialSetup.createInitialSetupAtNthPlatform(2));
      addMapping(DRCObstacleCourseStartingLocation.SMALL_WALL, SquaredUpDRCDemo01RobotAtPlatformsInitialSetup.createInitialSetupAtNthWall(2));
      addMapping(DRCObstacleCourseStartingLocation.SMALL_PLATFORM_TURNED, new OffsetAndYawRobotInitialSetup(-3.1, -4.73, 0.0, 3.0/4.0 * Math.PI));
      addMapping(DRCObstacleCourseStartingLocation.STAIRS, new SquaredUpDRCDemo01Robot(0.0, 1.9818, Math.toRadians(135.0), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.TOP_OF_STAIRS, new SquaredUpDRCDemo01Robot(0.60, 5.16, Math.toRadians(135.0), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.STEPPING_STONES, new SquaredUpDRCDemo01Robot(0.3, 7.2588, Math.toRadians(174.0695), Math.toRadians(5.0)));
      addMapping(DRCObstacleCourseStartingLocation.MID_LADDER, new SquaredUpDRCDemo01Robot(0.3, 7.1, Math.toRadians(45 * 3 / 2), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.LADDER, new SquaredUpDRCDemo01Robot(0.0, 6.5, Math.toRadians(45 * 3 / 2), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.SINGLE_CYLINDERBLOCKS, new SquaredUpDRCDemo01Robot(0.0, 6.0, Math.toRadians(-42.6959), Math.toRadians(90.0)));
      addMapping(DRCObstacleCourseStartingLocation.TOP_OF_SLOPES, new SquaredUpDRCDemo01Robot(0.155, 4.60, Math.toRadians(45), 0.0));
      addMapping(DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI, new SquaredUpDRCDemo01Robot(0.0, 0.0, Math.toRadians(170.0), 0));
   }

   private static void addMapping(DRCObstacleCourseStartingLocation drcDemo01StartingLocation, OffsetAndYawRobotInitialSetup robotInitialSetup)
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
      for (DRCObstacleCourseStartingLocation location : DRCObstacleCourseStartingLocation.values())
      {
         ret = ret + location.toString() + ", ";
      }
      ret = ret + "]";
      return ret;
   }
   
}
