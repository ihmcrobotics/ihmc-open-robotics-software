package us.ihmc.perception.sceneGraph;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import perception_msgs.msg.dds.SceneGraphMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.RequestConfirmFreezable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.filters.DetectionFilterCollection;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphTreeModification;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNodeTools;
import us.ihmc.perception.sceneGraph.topology.SceneGraphTopologyOperationQueue;
import us.ihmc.robotics.referenceFrames.ReferenceFrameDynamicCollection;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * A scene graph implementation that is really a scene tree. There
 * are no loops. This data structure is also a CRDT. It is synced between
 * the robot and operator UIs.
 *
 * The scene graph can be used to represent things a robot is looking for.
 */
public class SceneGraph extends RequestConfirmFreezable
{
   public static final double UPDATE_FREQUENCY = 60.0;

   /** The root node is always ID 0 and all nodes in the tree are unique. */
   public static long ROOT_NODE_ID = 0;
   public static String ROOT_NODE_NAME = "SceneGraphRoot";

   private final AtomicLong nextID = new AtomicLong(1); // Starts at 1 because root node is passed in
   private final SceneNode rootNode;
   private final SceneGraphTopologyOperationQueue topologyChangeQueue = new SceneGraphTopologyOperationQueue();
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
   private int numberOfFrozenNodes = 0;

   /** Create without CRDT synchronization. */
   public SceneGraph()
   {
      this(new SceneNode(ROOT_NODE_ID, ROOT_NODE_NAME), ROS2ActorDesignation.OPERATOR);
   }

   /**
    * Support passing in the externally created root node so it can be extended
    * by a superclass implementation like for UI nodes.
    */
   public SceneGraph(SceneNode rootNode, ROS2ActorDesignation actorDesignation)
   {
      super(new CRDTInfo(actorDesignation, (int) UPDATE_FREQUENCY));

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

      DoorNodeTools.addDoorNodes(this);

      modifyTreeTopology(modificationQueue -> updateOnRobotOnly(rootNode, robotPelvisFrame, modificationQueue));
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

   /** Convenience method. */
   public void modifyTreeTopology(Consumer<SceneGraphModificationQueue> modifier)
   {

   }

   /**
    * Use with {@link #getTopologyChangeQueue()}.
    */
   public void modifyTreeTopology()
   {
      boolean atLeastOnePerformed = topologyChangeQueue.performAllQueuedOperations();

      if (atLeastOnePerformed)
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
      numberOfFrozenNodes = 0;
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

      if (node.isFrozen())
         ++numberOfFrozenNodes;

      for (SceneNode child : node.getChildren())
      {
         updateCaches(child);
      }
   }

   public void fromMessage(SceneGraphMessage message)
   {
      fromMessage(message.getConfirmableRequest());

      if (!isFrozen())
         nextID.set(message.getNextId());
   }

   public SceneNode getRootNode()
   {
      return rootNode;
   }

   public AtomicLong getNextID()
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

   public SceneGraphTopologyOperationQueue getTopologyChangeQueue()
   {
      return topologyChangeQueue;
   }

   public int getNumberOfFrozenNodes()
   {
      return numberOfFrozenNodes;
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
