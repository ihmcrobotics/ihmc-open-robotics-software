package us.ihmc.rdx.simulation.environment;

import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;

public class RDXCustomSceneLoader
{
   public enum RDXDemoScene
   {
      FLAT_GROUND,
      EMPTY_ROOM,
      ROUGH_TERRAIN,
      EXPLOSIVE_BREACHING,
      ROOMS,
      /** A scanned multi-story house with its entrance in front of the robot. Needs scripts/hm3d_to_rdx.py run locally. */
      HM3D_BUILDING;
   }

   public static String getEnvironmentName(RDXDemoScene demoScene)
   {
      return switch (demoScene)
      {
         case EXPLOSIVE_BREACHING -> "BreachingDemo.json";
         case ROOMS -> "Rooms.json";
         case ROUGH_TERRAIN -> "StairsAndCinderBlockPath.json";
         case EMPTY_ROOM -> "EmptyRoom.json";
         case HM3D_BUILDING -> "HM3DBuilding.json";
         default -> "FlatGround.json";
      };
   }

   public static CommonAvatarEnvironmentInterface getEnvironment(RDXDemoScene demoScene)
   {
      return switch (demoScene)
      {
         default -> new FlatGroundEnvironment();
      };
   }
}