package us.ihmc.perception.sceneGraph;

import gnu.trove.map.TIntDoubleMap;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TIntDoubleHashMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameDynamicCollection;

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

   private final MutableLong nextID = new MutableLong();
   private final SceneNode rootNode;
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<SceneNode> idToNodeMap = new TLongObjectHashMap<>();
   private transient final List<String> nodeNameList = new ArrayList<>();
   private transient final Map<String, SceneNode> namesToNodesMap = new HashMap<>();
   private transient final TIntDoubleMap arUcoMarkerIDsToSizesMap = new TIntDoubleHashMap();

   public SceneGraph()
   {
      this(SceneNode::new);
   }

   public SceneGraph(BiFunction<Long, String, SceneNode> newRootNodeSupplier)
   {
      rootNode = newRootNodeSupplier.apply(nextID.getAndIncrement(), "SceneGraphRoot");
   }

   public void updateOnRobot(ReferenceFrame sensorFrame)
   {
      updateCaches();

      updateOnRobot(rootNode, sensorFrame);
   }

   private void updateOnRobot(SceneNode sceneNode, ReferenceFrame sensorFrame)
   {
      if (sceneNode instanceof StaticRelativeSceneNode staticRelativeSceneNode)
      {
         staticRelativeSceneNode.updateTrackingState(sensorFrame);
      }

      for (SceneNode child : sceneNode.getChildren())
      {
         updateOnRobot(child, sensorFrame);
      }
//      for (int i = 0; i < sceneNode.getChildren().size(); i++)
//      {
//         updateOnRobot(sceneNode.getChildren().get(i), sensorFrame);
//      }
   }

   public void ensureFramesMatchParentsRecursively()
   {
      rootNode.ensureFramesMatchParentsRecursively(ReferenceFrame.getWorldFrame());
   }

   public void updateCaches()
   {
      idToNodeMap.clear();
      nodeNameList.clear();
      namesToNodesMap.clear();
      arUcoMarkerIDsToSizesMap.clear();
      updateCaches(rootNode);
   }

   private void updateCaches(SceneNode node)
   {
      idToNodeMap.put(node.getID(), node);
      nodeNameList.add(node.getName());
      namesToNodesMap.put(node.getName(), node);

      if (node instanceof ArUcoMarkerNode arUcoMarkerNode)
      {
         arUcoMarkerIDsToSizesMap.put(arUcoMarkerNode.getMarkerID(), arUcoMarkerNode.getMarkerSize());
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

   public TIntDoubleMap getArUcoMarkerIDsToSizesMap()
   {
      return arUcoMarkerIDsToSizesMap;
   }

   public ReferenceFrameDynamicCollection asNewDynamicReferenceFrameCollection()
   {
      Function<String, ReferenceFrame> frameLookup = nodeName -> namesToNodesMap.get(nodeName).getNodeFrame();
      return new ReferenceFrameDynamicCollection(nodeNameList, frameLookup);
   }
}
