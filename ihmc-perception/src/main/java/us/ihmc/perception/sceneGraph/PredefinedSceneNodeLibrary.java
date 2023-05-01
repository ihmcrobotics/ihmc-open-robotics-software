package us.ihmc.perception.sceneGraph;

import gnu.trove.map.hash.TIntDoubleHashMap;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.sceneGraph.rigidBodies.RigidBodySceneObjectDefinitions;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticArUcoRelativeDetectableSceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;

import java.util.*;

/**
 * Use to specify which scene nodes a robot is looking for.
 *
 * We can put more specific stuff in here for now, like door specific
 * and even dynamic nodes.
 *
 * This is a super high level class, it includes ROS 2 topics,
 * heuristic stuff for certain nodes, stuff like that.
 *
 * This class exists so we can support multiple robots detecting the same
 * nodes, but also to serve the need we have to define things
 * by hand and construct custom heuristics.
 */
public class PredefinedSceneNodeLibrary
{
   private final ArUcoDetectableNode pushDoorPanel;
   private final ArUcoDetectableNode pullDoorPanel;
   private final ArUcoDetectableNode pushDoorLeverHandle;
   private final ArUcoDetectableNode pullDoorLeverHandle;
   private final ArUcoDetectableNode box;
   private final ArUcoDetectableNode canOfSoup;

   private final ArrayList<DetectableSceneNode> detectableSceneNodes = new ArrayList<>();
   private final ArrayList<ArUcoDetectableNode> arUcoDetectableNodes = new ArrayList<>();
   private final HashMap<Integer, StaticArUcoRelativeDetectableSceneNode> staticArUcoRelativeDetectableNodes = new HashMap<>();
   private final TIntDoubleHashMap arUcoMarkerIDsToSizes = new TIntDoubleHashMap();
   private final List<ReferenceFrame> referenceFrames = new ArrayList<>();

   public static PredefinedSceneNodeLibrary defaultObjects()
   {
      return new PredefinedSceneNodeLibrary();
   }

   private PredefinedSceneNodeLibrary()
   {
      // Add door stuff
      pushDoorPanel = DoorSceneNodeDefinitions.createPushDoorPanel();
      pullDoorPanel = DoorSceneNodeDefinitions.createPullDoorPanel();
      pushDoorLeverHandle = DoorSceneNodeDefinitions.createPushDoorLeverHandle();
      pullDoorLeverHandle = DoorSceneNodeDefinitions.createPullDoorLeverHandle();
      registerArUcoDetectableSceneNode(pushDoorPanel);
      registerArUcoDetectableSceneNode(pullDoorPanel);
      registerArUcoDetectableSceneNode(pushDoorLeverHandle);
      registerArUcoDetectableSceneNode(pullDoorLeverHandle);

      box = RigidBodySceneObjectDefinitions.createBox();
      canOfSoup = RigidBodySceneObjectDefinitions.createCanOfSoup();
      registerArUcoDetectableSceneNode(box);
      registerArUcoDetectableSceneNode(canOfSoup);

      // TODO: Remove aruco detectables and use non-ArUco objects -- detected by neural net

   }

   public void registerArUcoDetectableSceneNode(ArUcoDetectableNode arUcoDetectableNode)
   {
      registerDetectableSceneNode(arUcoDetectableNode);
      arUcoDetectableNodes.add(arUcoDetectableNode);
      arUcoMarkerIDsToSizes.put(arUcoDetectableNode.getMarkerID(), arUcoDetectableNode.getMarkerSize());
   }

   public void registerStaticArUcoRelativeDetectableSceneNode(StaticArUcoRelativeDetectableSceneNode staticArUcoRelativeDetectableSceneNode)
   {
      registerDetectableSceneNode(staticArUcoRelativeDetectableSceneNode);
      staticArUcoRelativeDetectableNodes.put(staticArUcoRelativeDetectableSceneNode.getMarkerID(), staticArUcoRelativeDetectableSceneNode);
   }

   public void registerDetectableSceneNode(DetectableSceneNode detectableSceneNode)
   {
      detectableSceneNodes.add(detectableSceneNode);
      referenceFrames.add(detectableSceneNode.getNodeFrame());
   }

   public List<DetectableSceneNode> getDetectableSceneNodes()
   {
      return detectableSceneNodes;
   }

   public List<ArUcoDetectableNode> getArUcoDetectableNodes()
   {
      return arUcoDetectableNodes;
   }

   public HashMap<Integer, StaticArUcoRelativeDetectableSceneNode> getStaticArUcoRelativeDetectableNodes()
   {
      return staticArUcoRelativeDetectableNodes;
   }

   public TIntDoubleHashMap getArUcoMarkerIDsToSizes()
   {
      return arUcoMarkerIDsToSizes;
   }

   public List<ReferenceFrame> getReferenceFrames()
   {
      return referenceFrames;
   }
}
