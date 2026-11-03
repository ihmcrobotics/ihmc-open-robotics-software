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
      ROOMS;
   }

   public static String getEnvironmentName(RDXDemoScene demoScene)
   {
      return switch (demoScene)
      {
         case EXPLOSIVE_BREACHING -> "BreachingDemo.json";
         case ROOMS -> "Rooms.json";
         case ROUGH_TERRAIN -> "StairsAndCinderBlockPath.json";
         case EMPTY_ROOM -> "EmptyRoom.json";
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