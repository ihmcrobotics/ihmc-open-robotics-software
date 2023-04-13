package us.ihmc.perception.scene;

import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.objects.BasicSceneObjects;
import us.ihmc.perception.objects.DoorSceneObjects;

import java.util.ArrayList;
import java.util.List;

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
public class PredefinedSceneObjectLibrary
{
   private final ArUcoDetectableObject pushDoorPanel;
   private final ArUcoDetectableObject pushDoorFrame;
   private final ArUcoDetectableObject pullDoorPanel;
   private final ArUcoDetectableObject pullDoorFrame;
   private final ArUcoDetectableObject box;
   private final ArUcoDetectableObject canOfSoup;

   private final ArrayList<ArUcoDetectableObject> arUcoDetectableObjects = new ArrayList<>();

   public static PredefinedSceneObjectLibrary defaultObjects()
   {
      return new PredefinedSceneObjectLibrary();
   }

   private PredefinedSceneObjectLibrary()
   {
      // Add door stuff
      pushDoorPanel = DoorSceneObjects.createPushDoorPanel();
      arUcoDetectableObjects.add(pushDoorPanel);
      pushDoorFrame = DoorSceneObjects.createPushDoorFrame();
      arUcoDetectableObjects.add(pushDoorFrame);
      pullDoorPanel = DoorSceneObjects.createPullDoorPanel();
      arUcoDetectableObjects.add(pullDoorPanel);
      pullDoorFrame = DoorSceneObjects.createPullDoorFrame();
      arUcoDetectableObjects.add(pullDoorFrame);

      box = BasicSceneObjects.createBox();
      arUcoDetectableObjects.add(box);
      canOfSoup = BasicSceneObjects.createCanOfSoup();
      arUcoDetectableObjects.add(canOfSoup);

      // Add non-ArUco cup -- detected by neural net

   }

   public ArUcoDetectableObject getPullDoorFrame()
   {
      return pullDoorFrame;
   }

   public ArUcoDetectableObject getPullDoorPanel()
   {
      return pullDoorPanel;
   }

   public ArUcoDetectableObject getPushDoorFrame()
   {
      return pushDoorFrame;
   }

   public ArUcoDetectableObject getPushDoorPanel()
   {
      return pushDoorPanel;
   }

   public List<ArUcoDetectableObject> getArUcoDetectableObjects()
   {
      return arUcoDetectableObjects;
   }
}
