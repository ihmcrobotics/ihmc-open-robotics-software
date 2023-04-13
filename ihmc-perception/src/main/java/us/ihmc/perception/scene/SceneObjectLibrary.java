package us.ihmc.perception.scene;

import us.ihmc.perception.objects.BasicSceneObjects;
import us.ihmc.perception.objects.DoorSceneObjects;

/**
 * Use to specify which scene objects a robot is looking for.
 *
 * We can put more specific stuff in here for now, like door specific
 * and even dynamic objects.
 *
 * This is a super high level class, it includes ROS 2 topics,
 * heuristic stuff for certain objects, stuff like that.
 *
 * This class exists so we can support multiple robots detecting the same
 * objects, but also to serve the need we have to define things
 * by hand and construct custom heuristics.
 */
public class SceneObjectLibrary
{
   private static ArUcoDetectableObject pushDoorPanel;
   private static ArUcoDetectableObject pushDoorFrame;
   private static ArUcoDetectableObject pullDoorPanel;
   private static ArUcoDetectableObject pullDoorFrame;
   private static ArUcoDetectableObject box;
   private static ArUcoDetectableObject canOfSoup;

   public static SceneObjectLibrary defaultObjects()
   {
      SceneObjectLibrary sceneObjectLibrary = new SceneObjectLibrary();

      // Add door stuff
      pushDoorPanel = DoorSceneObjects.createPushDoorPanel();
      pushDoorFrame = DoorSceneObjects.createPushDoorFrame();
      pullDoorPanel = DoorSceneObjects.createPullDoorPanel();
      pullDoorFrame = DoorSceneObjects.createPullDoorFrame();

      box = BasicSceneObjects.createBox();
      canOfSoup = BasicSceneObjects.createCanOfSoup();

      // Add non-ArUco cup -- detected by neural net

      return sceneObjectLibrary;
   }


}
