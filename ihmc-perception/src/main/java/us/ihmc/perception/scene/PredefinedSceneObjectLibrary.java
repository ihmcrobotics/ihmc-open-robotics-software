package us.ihmc.perception.scene;

import gnu.trove.map.hash.TIntDoubleHashMap;
import us.ihmc.perception.objects.BasicSceneObjects;
import us.ihmc.perception.objects.DoorSceneObjects;
import us.ihmc.perception.objects.StaticArUcoRelativeDetectableSceneObject;

import java.util.*;

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
   private final ArUcoDetectableObject pushDoorLeverHandle;
   private final ArUcoDetectableObject pullDoorLeverHandle;
   private final StaticArUcoRelativeDetectableSceneObject pushDoorFrame;
   private final StaticArUcoRelativeDetectableSceneObject pullDoorFrame;
   private final ArUcoDetectableObject box;
   private final ArUcoDetectableObject canOfSoup;

   private final HashSet<DetectableSceneObject> detectableSceneObjects = new HashSet<>();
   private final HashSet<ArUcoDetectableObject> arUcoDetectableObjects = new HashSet<>();
   private final HashMap<Integer, StaticArUcoRelativeDetectableSceneObject> staticArUcoRelativeDetectableObjects = new HashMap<>();
   private final TIntDoubleHashMap arUcoMarkerIDsToSizes = new TIntDoubleHashMap();

   public static PredefinedSceneObjectLibrary defaultObjects()
   {
      return new PredefinedSceneObjectLibrary();
   }

   private PredefinedSceneObjectLibrary()
   {
      // Add door stuff
      pushDoorPanel = DoorSceneObjects.createPushDoorPanel();
      pullDoorPanel = DoorSceneObjects.createPullDoorPanel();
      pushDoorLeverHandle = DoorSceneObjects.createPushDoorLeverHandle();
      pullDoorLeverHandle = DoorSceneObjects.createPullDoorLeverHandle();
      registerArUcoDetectableSceneObject(pushDoorPanel);
      registerArUcoDetectableSceneObject(pullDoorPanel);
      registerArUcoDetectableSceneObject(pushDoorLeverHandle);
      registerArUcoDetectableSceneObject(pullDoorLeverHandle);

      // The frames stay in place after being seen
      pushDoorFrame = DoorSceneObjects.createPushDoorFrame();
      pullDoorFrame = DoorSceneObjects.createPullDoorFrame();
      registerStaticArUcoRelativeDetectableSceneObject(pushDoorFrame);
      registerStaticArUcoRelativeDetectableSceneObject(pullDoorFrame);

      box = BasicSceneObjects.createBox();
      canOfSoup = BasicSceneObjects.createCanOfSoup();
      registerArUcoDetectableSceneObject(box);
      registerArUcoDetectableSceneObject(canOfSoup);

      // TODO: Add non-ArUco cup -- detected by neural net

   }

   public void registerArUcoDetectableSceneObject(ArUcoDetectableObject arUcoDetectableObject)
   {
      detectableSceneObjects.add(arUcoDetectableObject);
      arUcoDetectableObjects.add(arUcoDetectableObject);
      arUcoMarkerIDsToSizes.put(arUcoDetectableObject.getMarkerID(), arUcoDetectableObject.getMarkerSize());
   }

   public void registerStaticArUcoRelativeDetectableSceneObject(StaticArUcoRelativeDetectableSceneObject staticArUcoRelativeDetectableSceneObject)
   {
      detectableSceneObjects.add(staticArUcoRelativeDetectableSceneObject);
      staticArUcoRelativeDetectableObjects.put(staticArUcoRelativeDetectableSceneObject.getMarkerID(), staticArUcoRelativeDetectableSceneObject);
   }

   public HashSet<DetectableSceneObject> getDetectableSceneObjects()
   {
      return detectableSceneObjects;
   }

   public Set<ArUcoDetectableObject> getArUcoDetectableObjects()
   {
      return arUcoDetectableObjects;
   }

   public HashMap<Integer, StaticArUcoRelativeDetectableSceneObject> getStaticArUcoRelativeDetectableObjects()
   {
      return staticArUcoRelativeDetectableObjects;
   }

   public TIntDoubleHashMap getArUcoMarkerIDsToSizes()
   {
      return arUcoMarkerIDsToSizes;
   }
}
