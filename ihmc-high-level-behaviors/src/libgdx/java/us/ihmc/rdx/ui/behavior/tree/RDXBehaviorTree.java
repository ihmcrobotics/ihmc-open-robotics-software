package us.ihmc.rdx.ui.behavior.tree;

import com.fasterxml.jackson.databind.JsonNode;
import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeDestroySubtree;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeModification;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeModificationQueue;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeNodeAddition;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.*;
import java.util.function.Consumer;

public class RDXBehaviorTree
{
   private final BehaviorTreeState behaviorTreeState = new BehaviorTreeState();
   private final RDXBehaviorTreeNodeBuilder nodeBuilder;
   private RDXBehaviorTreeNode rootNode;
   private final Queue<RDXBehaviorTreeModification> queuedModifications = new LinkedList<>();
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<RDXBehaviorTreeNode> idToNodeMap = new TLongObjectHashMap<>();

   public RDXBehaviorTree(DRCRobotModel robotModel,
                          ROS2SyncedRobotModel syncedRobot,
                          RobotCollisionModel selectionCollisionModel,
                          RDXBaseUI baseUI,
                          RDX3DPanel panel3D,
                          ReferenceFrameLibrary referenceFrameLibrary,
                          ROS2ControllerPublishSubscribeAPI ros2)
   {
      // TODO: Do we create the publishers and subscribers here?

      nodeBuilder = new RDXBehaviorTreeNodeBuilder(robotModel, syncedRobot, selectionCollisionModel, baseUI, panel3D, referenceFrameLibrary, ros2);
   }

   public void loadFromFile()
   {
      WorkspaceResourceFile file = null; // FIXME

      // Delete the entire tree. We are starting over
      modifyTree(modificationQueue ->
      {
         modificationQueue.accept(new RDXBehaviorTreeDestroySubtree(rootNode));

         JSONFileTools.load(file, jsonNode ->
         {
            rootNode = loadFromFile(jsonNode, null, modificationQueue);
         });
      });
   }

   public RDXBehaviorTreeNode loadFromFile(JsonNode jsonNode, RDXBehaviorTreeNode parentNode, RDXBehaviorTreeModificationQueue modificationQueue)
   {
      String typeName = jsonNode.get("type").textValue();

      RDXBehaviorTreeNode node = nodeBuilder.createNode(RDXBehaviorTreeTools.getClassFromTypeName(typeName), behaviorTreeState.getNextID().getAndIncrement());

      node.getDefinition().loadFromFile(jsonNode);

      if (parentNode != null)
      {
         modificationQueue.accept(new RDXBehaviorTreeNodeAddition(node, parentNode));
      }

      JSONTools.forEachArrayElement(jsonNode, "children", childJsonNode ->
      {
         loadFromFile(childJsonNode, node, modificationQueue);
      });

      return node;
   }

   public void modifyTree(Consumer<RDXBehaviorTreeModificationQueue> modifier)
   {
      modifier.accept(queuedModifications::add);

      boolean modified = !queuedModifications.isEmpty();

      while (!queuedModifications.isEmpty())
      {
         RDXBehaviorTreeModification modification = queuedModifications.poll();
         modification.performOperation();
      }

      if (modified)
         update();
   }

   private void update()
   {
      idToNodeMap.clear();
      updateCaches(rootNode);
   }

   private void updateCaches(RDXBehaviorTreeNode node)
   {
      idToNodeMap.put(node.getState().getID(), node);

      for (RDXBehaviorTreeNode child : node.getChildren())
      {
         updateCaches(child);
      }
   }

   public TLongObjectMap<RDXBehaviorTreeNode> getIDToNodeMap()
   {
      return idToNodeMap;
   }
}
