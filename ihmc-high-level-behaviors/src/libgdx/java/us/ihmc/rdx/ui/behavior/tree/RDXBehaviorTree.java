package us.ihmc.rdx.ui.behavior.tree;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateModification;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeDestroySubtree;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeModificationQueue;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeNodeAddition;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.*;
import java.util.function.Consumer;

public class RDXBehaviorTree
{
   /** The root node is always ID 0 and all nodes in the tree are unique. */
   public static long ROOT_NODE_ID = 0;

   private final MutableLong nextID = new MutableLong(1); // Starts at 1 because root node is created automatically
   private RDXBehaviorTreeNode rootNode;
   private final Queue<BehaviorTreeStateModification> queuedModifications = new LinkedList<>();
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<RDXBehaviorTreeNode> idToNodeMap = new TLongObjectHashMap<>();

   public RDXBehaviorTree()
   {

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
            String typeName = jsonNode.get("type").textValue();

            rootNode = RDXBehaviorTreeTools.createNode(RDXBehaviorTreeTools.getClassFromTypeName(typeName), nextID.getAndIncrement());

            // load children

         });


      });


   }

   public void modifyTree(Consumer<RDXBehaviorTreeModificationQueue> modifier)
   {
      modifier.accept(queuedModifications::add);

      boolean modified = !queuedModifications.isEmpty();

      while (!queuedModifications.isEmpty())
      {
         BehaviorTreeStateModification modification = queuedModifications.poll();
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
      idToNodeMap.put(node.getID(), node);

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
