package us.ihmc.perception.scene;

import us.ihmc.perception.OpenCVArUcoMarker;
import us.ihmc.perception.objects.BasicSceneObjects;
import us.ihmc.perception.objects.DoorSceneObjects;
import us.ihmc.perception.objects.StaticArUcoRelativeDetectableSceneObject;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

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
   private final ArUcoDetectableObject pullDoorPanel;
   private final StaticArUcoRelativeDetectableSceneObject pushDoorFrame;
   private final StaticArUcoRelativeDetectableSceneObject pullDoorFrame;
   private final ArUcoDetectableObject box;
   private final ArUcoDetectableObject canOfSoup;

   private final HashSet<ArUcoDetectableObject> arUcoDetectableObjects = new HashSet<>();
   private final HashSet<StaticArUcoRelativeDetectableSceneObject> staticArUcoRelativeDetectableObjects = new HashSet<>();

   public static PredefinedSceneObjectLibrary defaultObjects()
   {
      return new PredefinedSceneObjectLibrary();
   }

   private PredefinedSceneObjectLibrary()
   {
      // Add door stuff
      pushDoorPanel = DoorSceneObjects.createPushDoorPanel();
      pullDoorPanel = DoorSceneObjects.createPullDoorPanel();
      arUcoDetectableObjects.add(pushDoorPanel);
      arUcoDetectableObjects.add(pullDoorPanel);

      // The frames stay in place after being seen
      pushDoorFrame = DoorSceneObjects.createPushDoorFrame();
      pullDoorFrame = DoorSceneObjects.createPullDoorFrame();
      staticArUcoRelativeDetectableObjects.add(pushDoorFrame);
      staticArUcoRelativeDetectableObjects.add(pullDoorFrame);

      box = BasicSceneObjects.createBox();
      canOfSoup = BasicSceneObjects.createCanOfSoup();
      arUcoDetectableObjects.add(box);
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

   public Set<ArUcoDetectableObject> getArUcoDetectableObjects()
   {
      return arUcoDetectableObjects;
   }

   public Set<StaticArUcoRelativeDetectableSceneObject> getStaticArUcoRelativeDetectableObjects()
   {
      return staticArUcoRelativeDetectableObjects;
   }
}
