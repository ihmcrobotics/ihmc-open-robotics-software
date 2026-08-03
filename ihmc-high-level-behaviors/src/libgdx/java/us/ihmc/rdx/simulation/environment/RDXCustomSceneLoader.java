package us.ihmc.rdx.simulation.environment;

import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;

public class RDXCustomSceneLoader
{
   public enum RDXDemoScene
   {
      FLAT_GROUND,
      ROUGH_TERRAIN,
      EXPLOSIVE_BREACHING,
      DISPOSE_BOTTLE,
      ROOM_WITH_OBJECTS,
      MANIPULATION_2X4,
      NAVIGATION_BARRIER;
   }

   public static String getEnvironmentName(RDXDemoScene demoScene)
   {
      return switch (demoScene)
      {
         case EXPLOSIVE_BREACHING -> "BreachingDemo.json";
         case DISPOSE_BOTTLE -> "DisposeBottle.json";
         case ROOM_WITH_OBJECTS -> "RoomWithObjects.json";
         case ROUGH_TERRAIN -> "HarderTerrain.json";
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