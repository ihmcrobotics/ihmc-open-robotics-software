package us.ihmc.rdx.ui.behavior.tree;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateModification;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeDestroySubtree;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeModificationQueue;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.*;
import java.util.function.Consumer;
import java.util.function.Function;

public class RDXBehaviorTree
{
   /** The root node is always ID 0 and all nodes in the tree are unique. */
   public static long ROOT_NODE_ID = 0;

   private final MutableLong nextID = new MutableLong(1); // Starts at 1 because root node is created automatically
   private RDXBehaviorTreeNode rootNode;
   private final Function<Class<?>, RDXBehaviorTreeNode> newNodeSupplier;
   private final Queue<BehaviorTreeStateModification> queuedModifications = new LinkedList<>();
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<BehaviorTreeNodeState> idToNodeMap = new TLongObjectHashMap<>();

   public RDXBehaviorTree(Function<Class<?>, RDXBehaviorTreeNode> newNodeSupplier)
   {
      this.newNodeSupplier = newNodeSupplier;
   }

   public void loadFromFile()
   {
      WorkspaceResourceFile file = null; // FIXME

      // Delete the entire tree. We are starting over
      modifyTree(modificationQueue -> new RDXBehaviorTreeDestroySubtree(rootNode));

      JSONFileTools.load(file, jsonNode ->
      {
         String typeName = jsonNode.get("type").textValue();

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

   private void updateCaches(BehaviorTreeNodeState node)
   {
      idToNodeMap.put(node.getID(), node);

      for (BehaviorTreeNodeState child : node.getChildren())
      {
         updateCaches(child);
      }
   }

   public TLongObjectMap<BehaviorTreeNodeState> getIDToNodeMap()
   {
      return idToNodeMap;
   }
}
