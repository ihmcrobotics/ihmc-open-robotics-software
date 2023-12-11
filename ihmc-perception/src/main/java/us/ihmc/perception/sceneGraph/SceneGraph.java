package us.ihmc.perception.sceneGraph;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.filters.DetectionFilterCollection;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphTreeModification;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameDynamicCollection;

import java.util.*;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * A scene graph implementation that is really a scene tree. There
 * are no loops. This data structure is also a CRDT. It is synced between
 * the robot and operator UIs.
 *
 * The scene graph can be used to represent things a robot is looking for.
 */
public class SceneGraph
{
   /** The root node is always ID 0 and all nodes in the tree are unique. */
   public static long ROOT_NODE_ID = 0;
   public static String ROOT_NODE_NAME = "SceneGraphRoot";

   private final MutableLong nextID = new MutableLong(1); // Starts at 1 because root node is passed in
   private final SceneNode rootNode;
   private final Queue<SceneGraphTreeModification> queuedModifications = new LinkedList<>();
   private final DetectionFilterCollection detectionFilterCollection = new DetectionFilterCollection();
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<SceneNode> idToNodeMap = new TLongObjectHashMap<>();
   private transient final List<String> nodeNameList = new ArrayList<>();
   private transient final Map<String, SceneNode> namesToNodesMap = new HashMap<>();
   private transient final TIntObjectMap<ArUcoMarkerNode> arUcoMarkerIDToNodeMap = new TIntObjectHashMap<>();
   private transient final TIntObjectMap<CenterposeNode> centerposeDetectedMarkerIDToNodeMap = new TIntObjectHashMap<>();
   private transient final SortedSet<SceneNode> sceneNodesByID = new TreeSet<>(Comparator.comparingLong(SceneNode::getID));

   public SceneGraph()
   {
      this(new SceneNode(ROOT_NODE_ID, ROOT_NODE_NAME));
   }

   /**
    * Support passing in the externally created root node so it can be extended
    * by a superclass implementation like for UI nodes.
    */
   public SceneGraph(SceneNode rootNode)
   {
      this.rootNode = rootNode;
      updateCaches(rootNode);
   }

   /**
    * This method should only be called on the robot's copy of the scene graph.
    * It should be called exactly once per frame and after perceptual detections
    * have been collected, like ArUco marker detections.
    *
    * This method updates the caches and the static relative nodes, whose
    * tracking state should only be evaluated by the robot.
    */
   public void updateOnRobotOnly(ReferenceFrame robotPelvisFrame)
   {
      // This must happen only once per on-robot tick
      detectionFilterCollection.update();

      modifyTree(modificationQueue -> updateOnRobotOnly(rootNode, robotPelvisFrame, modificationQueue));
   }

   private void updateOnRobotOnly(SceneNode sceneNode, ReferenceFrame sensorFrame, SceneGraphModificationQueue modificationQueue)
   {
      if (sceneNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
      {
         staticRelativeSceneNode.updateTrackingState(sensorFrame, modificationQueue);
      }

      for (SceneNode child : sceneNode.getChildren())
      {
         updateOnRobotOnly(child, sensorFrame, modificationQueue);
      }
   }

   public void modifyTree(Consumer<SceneGraphModificationQueue> modifier)
   {
      modifier.accept(queuedModifications::add);

      boolean modified = !queuedModifications.isEmpty();

      while (!queuedModifications.isEmpty())
      {
         SceneGraphTreeModification modification = queuedModifications.poll();
         modification.performOperation();
      }

      if (modified)
         update();
   }

   private void update()
   {
      idToNodeMap.clear();
      synchronized (nodeNameList)
      {
         nodeNameList.clear();
      }
      namesToNodesMap.clear();
      arUcoMarkerIDToNodeMap.clear();
      centerposeDetectedMarkerIDToNodeMap.clear();
      sceneNodesByID.clear();
      updateCaches(rootNode);
   }

   private void updateCaches(SceneNode node)
   {
      idToNodeMap.put(node.getID(), node);
      synchronized (nodeNameList)
      {
         nodeNameList.add(node.getName());
      }
      namesToNodesMap.put(node.getName(), node);
      sceneNodesByID.add(node);

      if (node instanceof ArUcoMarkerNode arUcoMarkerNode)
      {
         arUcoMarkerIDToNodeMap.put(arUcoMarkerNode.getMarkerID(), arUcoMarkerNode);
      }
      else if (node instanceof CenterposeNode centerposeNode)
      {
         centerposeDetectedMarkerIDToNodeMap.put(centerposeNode.getObjectID(), centerposeNode);
      }

      for (SceneNode child : node.getChildren())
      {
         updateCaches(child);
      }
   }

   public SceneNode getRootNode()
   {
      return rootNode;
   }

   public MutableLong getNextID()
   {
      return nextID;
   }

   public DetectionFilterCollection getDetectionFilterCollection()
   {
      return detectionFilterCollection;
   }

   public TLongObjectMap<SceneNode> getIDToNodeMap()
   {
      return idToNodeMap;
   }

   public List<String> getNodeNameList()
   {
      return nodeNameList;
   }

   public Map<String, SceneNode> getNamesToNodesMap()
   {
      return namesToNodesMap;
   }

   public TIntObjectMap<ArUcoMarkerNode> getArUcoMarkerIDToNodeMap()
   {
      return arUcoMarkerIDToNodeMap;
   }

   public TIntObjectMap<CenterposeNode> getCenterposeDetectedMarkerIDToNodeMap()
   {
      return centerposeDetectedMarkerIDToNodeMap;
   }

   public SortedSet<SceneNode> getSceneNodesByID()
   {
      return sceneNodesByID;
   }

   public ReferenceFrameDynamicCollection asNewDynamicReferenceFrameCollection()
   {
      Function<String, ReferenceFrame> frameLookup = nodeName ->
      {
         SceneNode sceneNode = namesToNodesMap.get(nodeName);
         return sceneNode == null ? null : sceneNode.getNodeFrame();
      };
      return new ReferenceFrameDynamicCollection(nodeNameList, frameLookup);
   }
}
