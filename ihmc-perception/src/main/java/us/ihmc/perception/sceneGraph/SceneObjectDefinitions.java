package us.ihmc.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;

import java.util.function.Function;

/**
 * Constant definitions pertaining to all scene objects.
 */
public class SceneObjectDefinitions
{
   /**
    * This is the width of the markers printed with IHMC's large format
    * printer. Send ihmc-perception/src/main/resources/arUcoMarkers/Markers0Through3.pdf
    * to IT to get new ones printed.
    */
   public static final double LARGE_MARKER_WIDTH = 0.1982;

   public static final Function<Integer, Double> ARUCO_MARKER_SIZES = markerID ->
   {
      if (RigidBodySceneObjectDefinitions.ARUCO_MARKER_SIZES.containsKey(markerID))
      {
         return RigidBodySceneObjectDefinitions.ARUCO_MARKER_SIZES.get(markerID);
      }
      else if (DoorSceneNodeDefinitions.ARUCO_MARKER_SIZES.containsKey(markerID))
      {
         return DoorSceneNodeDefinitions.ARUCO_MARKER_SIZES.get(markerID);
      }
      else
      {
         return LARGE_MARKER_WIDTH;
      }
   };
}
