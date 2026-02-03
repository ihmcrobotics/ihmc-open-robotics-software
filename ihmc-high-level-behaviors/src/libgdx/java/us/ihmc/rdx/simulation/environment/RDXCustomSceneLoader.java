package us.ihmc.rdx.simulation.environment;

public class RDXCustomSceneLoader
{
   public enum RDXDemoScene
   {
      FLAT_GROUND,
      ROUGH_TERRAIN,
      EXPLOSIVE_BREACHING_A,
      ROOM_WITH_OBJECTS,
      MANIPULATION_2X4,
      NAVIGATION_BARRIER;
   }

   public static String getEnvironmentName(RDXDemoScene demoScene)
   {
      return switch (demoScene)
      {
         case EXPLOSIVE_BREACHING_A -> "BreachingDemoA.json";
         case ROOM_WITH_OBJECTS -> "RoomWithObjects.json";
         case ROUGH_TERRAIN -> "HarderTerrain.json";
         default -> "FlatGround.json";
      };
   }
}