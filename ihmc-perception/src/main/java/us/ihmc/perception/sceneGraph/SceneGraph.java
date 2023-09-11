package us.ihmc.perception.sceneGraph;

import gnu.trove.map.TIntDoubleMap;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TIntDoubleHashMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameDynamicCollection;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
   private final MutableInt nextID = new MutableInt();
   private final SceneNode rootNode = new SceneNode(nextID.getAndIncrement(), "SceneGraphRoot");
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<SceneNode> idToNodeMap = new TLongObjectHashMap<>();
   private transient final List<String> nodeNameList = new ArrayList<>();
   private transient final Map<String, SceneNode> namesToNodesMap = new HashMap<>();
   private transient final TIntDoubleMap arUcoMarkerIDsToSizesMap = new TIntDoubleHashMap();
   private transient final FramePose3D detectedSceneNodePose = new FramePose3D();

   public SceneGraph()
   {
      // Here so that you can track the instantiations of this class with the IDE
   }

   public void update(ReferenceFrame sensorFrame)
   {
      updateCaches();

      rootNode.update(sceneNode ->
      {
         if (sceneNode instanceof DetectableSceneNode detectableSceneNode)
         {
            if (detectableSceneNode.getCurrentlyDetected())
            {
               for (SceneNode child : detectableSceneNode.getChildren())
               {
                  if (child instanceof StaticRelativeSceneNode staticRelativeSceneNode)
                  {
                     detectedSceneNodePose.setToZero(detectableSceneNode.getNodeFrame());
                     detectedSceneNodePose.setFromReferenceFrame(sensorFrame);
                     double currentDistance = detectedSceneNodePose.getPosition().distanceFromOrigin();
                     staticRelativeSceneNode.setCurrentDistance(currentDistance);
                     if (currentDistance <= staticRelativeSceneNode.getDistanceToDisableTracking())
                     {
                        staticRelativeSceneNode.setTrackInitialParent(false);
                     }
                  }
               }
            }
         }
      });
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

   public MutableInt getNextID()
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
