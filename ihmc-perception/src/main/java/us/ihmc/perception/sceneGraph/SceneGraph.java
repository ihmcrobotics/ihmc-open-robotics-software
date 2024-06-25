package us.ihmc.perception.sceneGraph;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.detections.PersistentDetection;
import us.ihmc.perception.detections.YOLOv8.YOLOv8InstantDetection;
import us.ihmc.perception.detections.centerPose.CenterPoseInstantDetection;
import us.ihmc.perception.filters.DetectionFilterCollection;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.perception.sceneGraph.modification.SceneGraphTreeModification;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.doors.DoorNodeTools;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8Node;
import us.ihmc.robotics.referenceFrames.ReferenceFrameDynamicCollection;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
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
public class SceneGraph
{
   /** This is the frequency the scene graph is expected to be updated on the robot. */
   public static final double UPDATE_FREQUENCY = 60.0;
   /** This is the frequency the CRDT syncs the scene graph state between operator and robot. */
   public static final double CRDT_SYNC_FREQUENCY = UPDATE_FREQUENCY / 2.0;

   /** The root node is always ID 0 and all nodes in the tree are unique. */
   public static long ROOT_NODE_ID = 0;
   public static String ROOT_NODE_NAME = "SceneGraphRoot";

   private final AtomicLong nextID = new AtomicLong(1); // Starts at 1 because root node is passed in
   private final SceneNode rootNode;
   private final Queue<SceneGraphTreeModification> queuedModifications = new LinkedList<>();
   private final DetectionFilterCollection detectionFilterCollection = new DetectionFilterCollection();
   private DetectionManager detectionManager = null;
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<SceneNode> idToNodeMap = new TLongObjectHashMap<>();
   private transient final List<String> nodeNameList = new ArrayList<>();
   private transient final Map<String, SceneNode> namesToNodesMap = new HashMap<>();
   private transient final TIntObjectMap<ArUcoMarkerNode> arUcoMarkerIDToNodeMap = new TIntObjectHashMap<>();
   private transient final SortedSet<SceneNode> sceneNodesByID = new TreeSet<>(Comparator.comparingLong(SceneNode::getID));
   private transient final Set<DetectableSceneNode> detectableSceneNodes = new HashSet<>();
   private int numberOfFrozenNodes = 0;

   /** Create without CRDT synchronization. */
   public SceneGraph()
   {
      this(new SceneNode(ROOT_NODE_ID, ROOT_NODE_NAME, new CRDTInfo(ROS2ActorDesignation.OPERATOR, (int) CRDT_SYNC_FREQUENCY)));
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

      DoorNodeTools.addDoorNodes(this);

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
      destroyRemovedSceneNodes();

      idToNodeMap.clear();
      synchronized (nodeNameList)
      {
         nodeNameList.clear();
      }
      namesToNodesMap.clear();
      arUcoMarkerIDToNodeMap.clear();
      sceneNodesByID.clear();
      detectableSceneNodes.clear();
      numberOfFrozenNodes = 0;
      updateCaches(rootNode);
   }

   private void destroyRemovedSceneNodes()
   {
      Set<Long> existingSceneNodeIDs = new HashSet<>();

      getExistingSceneNodeIDs(rootNode, existingSceneNodeIDs);

      for (long id : idToNodeMap.keys())
      {
         if (!existingSceneNodeIDs.contains(id))
            idToNodeMap.get(id).destroy(this);
      }
   }

   private void getExistingSceneNodeIDs(SceneNode node, Collection<Long> ids)
   {
      ids.add(node.getID());

      for (SceneNode childNode : node.getChildren())
      {
         getExistingSceneNodeIDs(childNode, ids);
      }
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

      if (node instanceof DetectableSceneNode detectableSceneNode)
      {
         detectableSceneNodes.add(detectableSceneNode);
      }

      if (node instanceof ArUcoMarkerNode arUcoMarkerNode)
      {
         arUcoMarkerIDToNodeMap.put(arUcoMarkerNode.getMarkerID(), arUcoMarkerNode);
      }

      if (node.isFrozen())
         ++numberOfFrozenNodes;

      for (SceneNode child : node.getChildren())
      {
         updateCaches(child);
      }
   }

   public void updateDetections()
   {
      if (detectionManager == null)
         return;

      Set<PersistentDetection<?>> detections = detectionManager.updateAndGetDetections();
      for (PersistentDetection<?> detection : detections)
      {
         boolean claimed = false;
         for (DetectableSceneNode node : detectableSceneNodes)
         {
            if (node.hasMatchingDetection(detection))
            {
               node.updateDetection(detection.getMostRecentDetection());
               node.setCurrentlyDetected(detection.isStable());
               node.update(this);
               claimed = true;
            }
         }

         if (!claimed && detection.isOldEnough())
         {
            if (detection.isStable())
               addNodeFromDetection(detection);
            else
               detection.markForDeletion();
         }
      }
   }

   public SceneNode getRootNode()
   {
      return rootNode;
   }

   public CRDTInfo getCRDTInfo()
   {
      return rootNode.getCRDTInfo();
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

   public SortedSet<SceneNode> getSceneNodesByID()
   {
      return sceneNodesByID;
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

   public void setDetectionManager(DetectionManager detectionManager)
   {
      this.detectionManager = detectionManager;
   }

   public DetectionManager getDetectionManager()
   {
      return detectionManager;
   }

   // Don't worry, I've got this
   private void addNodeFromDetection(PersistentDetection<? extends InstantDetection> detection)
   {
      DetectableSceneNode detectableNode;
      long newNodeID = getNextID().getAndIncrement();
      String newNodeName = detection.getDetectedObjectName() + newNodeID;

      Class<?> detectionClass = detection.getInstantDetectionClass();
      if (detectionClass.equals(YOLOv8InstantDetection.class))
         detectableNode = new YOLOv8Node(newNodeID, newNodeName, (YOLOv8InstantDetection) detection.getMostRecentDetection(), getCRDTInfo());
      else if (detectionClass.equals(CenterPoseInstantDetection.class))
         detectableNode = new CenterposeNode(newNodeID, newNodeName, (CenterPoseInstantDetection) detection.getMostRecentDetection(), true, getCRDTInfo());
      else
      {
         LogTools.error("Logic to handle detections of class {} has not been implemented", detectionClass);
         detectableNode = null;
      }

      if (detectableNode != null)
      {
         modifyTree(modificationQueue -> modificationQueue.accept(new SceneGraphNodeAddition(detectableNode, rootNode)));
         detectableSceneNodes.add(detectableNode);
      }
   }
}
