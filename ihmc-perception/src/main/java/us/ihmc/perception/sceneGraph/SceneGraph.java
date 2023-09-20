package us.ihmc.perception.sceneGraph;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.filters.DetectionFilterCollection;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphPublisher;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscription;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphSubscriptionNode;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraphTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameDynamicCollection;
import us.ihmc.tools.thread.Throttler;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiFunction;
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
   private final DetectionFilterCollection detectionFilterCollection = new DetectionFilterCollection();
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final ROS2IOTopicQualifier subscriptionQualifier;
   private ROS2SceneGraphSubscription sceneGraphSubscription;
   private final ROS2SceneGraphPublisher sceneGraphPublisher = new ROS2SceneGraphPublisher();
   private final Throttler publishThrottler = new Throttler().setFrequency(30.0);
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<SceneNode> idToNodeMap = new TLongObjectHashMap<>();
   private transient final List<String> nodeNameList = new ArrayList<>();
   private transient final Map<String, SceneNode> namesToNodesMap = new HashMap<>();
   private transient final TIntObjectMap<ArUcoMarkerNode> arUcoMarkerIDToNodeMap = new TIntObjectHashMap<>();
   private transient final List<SceneGraphNodeMove> sceneGraphNodeMoves = new ArrayList<>();

   /** Constuctor if you don't want ROS 2 sync. */
   public SceneGraph()
   {
      this(new SceneNode(ROOT_NODE_ID, ROOT_NODE_NAME), null, null, null);
   }

   /**
    * Constructor for on-robot.
    */
   public SceneGraph(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this(new SceneNode(ROOT_NODE_ID, ROOT_NODE_NAME),
           (sceneGraph, subscriptionNode) -> ROS2SceneGraphTools.createNodeFromMessage(subscriptionNode, sceneGraph),
           ros2PublishSubscribeAPI,
           ROS2IOTopicQualifier.COMMAND);
   }

   /**
    * The complexity of this constructor is to support the UI having nodes that extend the base
    * on-robot ones.
    */
   public SceneGraph(SceneNode rootNode,
                     BiFunction<SceneGraph, ROS2SceneGraphSubscriptionNode, SceneNode> newNodeSupplier,
                     ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                     ROS2IOTopicQualifier subscriptionQualifier)
   {
      this.rootNode = rootNode;
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;
      this.subscriptionQualifier = subscriptionQualifier;

      if (ros2PublishSubscribeAPI != null)
         sceneGraphSubscription = new ROS2SceneGraphSubscription(this, ros2PublishSubscribeAPI, subscriptionQualifier, newNodeSupplier);
   }

   public void updateSubscription()
   {
      if (sceneGraphSubscription != null)
         sceneGraphSubscription.update();
   }

   /**
    * This method updates the caches and the static relative nodes, whose
    * tracking state should only be evaluated by the robot.
    */
   public void updateOnRobot(ReferenceFrame sensorFrame)
   {
      updateCaches();
      sceneGraphNodeMoves.clear();
      detectionFilterCollection.update();

      updateOnRobot(rootNode, sensorFrame);

      for (SceneGraphNodeMove sceneGraphNodeMove : sceneGraphNodeMoves)
      {
         sceneGraphNodeMove.performMove();
      }
   }

   private void updateOnRobot(SceneNode sceneNode, ReferenceFrame sensorFrame)
   {
      if (sceneNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
      {
         staticRelativeSceneNode.updateTrackingState(sensorFrame, sceneGraphNodeMoves);
      }

      for (SceneNode child : sceneNode.getChildren())
      {
         updateOnRobot(child, sensorFrame);
      }
   }

   public void ensureFramesMatchParentsRecursively()
   {
      rootNode.ensureFramesMatchParentsRecursively(ReferenceFrame.getWorldFrame());
   }

   public void updatePublication()
   {
      if (ros2PublishSubscribeAPI != null && publishThrottler.run())
         sceneGraphPublisher.publish(this, ros2PublishSubscribeAPI, subscriptionQualifier.getOpposite());
   }

   public void updateCaches()
   {
      idToNodeMap.clear();
      nodeNameList.clear();
      namesToNodesMap.clear();
      arUcoMarkerIDToNodeMap.clear();
      updateCaches(rootNode);
   }

   private void updateCaches(SceneNode node)
   {
      idToNodeMap.put(node.getID(), node);
      nodeNameList.add(node.getName());
      namesToNodesMap.put(node.getName(), node);

      if (node instanceof ArUcoMarkerNode arUcoMarkerNode)
      {
         arUcoMarkerIDToNodeMap.put(arUcoMarkerNode.getMarkerID(), arUcoMarkerNode);
      }

      for (SceneNode child : node.getChildren())
      {
         updateCaches(child);
      }
   }

   public void destroy()
   {
      if (sceneGraphSubscription != null)
         sceneGraphSubscription.destroy();
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

   public List<SceneGraphNodeMove> getSceneGraphNodeMoves()
   {
      return sceneGraphNodeMoves;
   }

   public ReferenceFrameDynamicCollection asNewDynamicReferenceFrameCollection()
   {
      Function<String, ReferenceFrame> frameLookup = nodeName -> namesToNodesMap.get(nodeName).getNodeFrame();
      return new ReferenceFrameDynamicCollection(nodeNameList, frameLookup);
   }

   public ROS2SceneGraphSubscription getSceneGraphSubscription()
   {
      return sceneGraphSubscription;
   }
}
