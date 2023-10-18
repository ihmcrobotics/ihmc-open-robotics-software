package us.ihmc.behaviors.behaviorTree;

import gnu.trove.map.TLongObjectMap;
import gnu.trove.map.hash.TLongObjectHashMap;
import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModification;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;

import java.util.*;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * This is the state related functionality of a behavior tree,
 * which would live on the UI side and the robot side.
 *
 * The root node is going to be a single basic root node with no functionality
 * and it will never be replaced.
 */
public class BehaviorTreeState
{
   /** The root node is always ID 0 and all nodes in the tree are unique. */
   public static long ROOT_NODE_ID = 0;
   public static String ROOT_NODE_NAME = "BehaviorTreeRoot";

   private final MutableLong nextID = new MutableLong(1); // Starts at 1 because root node is created automatically
   private final BehaviorTreeRootNode rootNode = new BehaviorTreeRootNode();
   private final Function<Class<?>, BehaviorTreeNodeState> newNodeSupplier;
   private final Queue<BehaviorTreeModification> queuedModifications = new LinkedList<>();
   /**
    * Useful for accessing nodes by ID instead of searching.
    * Also, sometimes, the tree will be disassembled and this is used in putting it
    * back together.
    */
   private transient final TLongObjectMap<BehaviorTreeNodeState> idToNodeMap = new TLongObjectHashMap<>();
   private transient final List<String> nodeNameList = new ArrayList<>();
   private transient final Map<String, BehaviorTreeNodeState> namesToNodesMap = new HashMap<>();
   private transient final SortedSet<BehaviorTreeNodeState> nodesByID = new TreeSet<>(Comparator.comparingLong(BehaviorTreeNodeState::getID));

   public BehaviorTreeState(Function<Class<?>, BehaviorTreeNodeState> newNodeSupplier)
   {
      this.newNodeSupplier = newNodeSupplier;
   }

   public void modifyTree(Consumer<BehaviorTreeModificationQueue> modifier)
   {
      modifier.accept(queuedModifications::add);

      boolean modified = !queuedModifications.isEmpty();

      while (!queuedModifications.isEmpty())
      {
         BehaviorTreeModification modification = queuedModifications.poll();
         modification.performOperation();
      }

      if (modified)
         update();
   }

   private void update()
   {
      idToNodeMap.clear();
      nodeNameList.clear();
      namesToNodesMap.clear();
      nodesByID.clear();
      updateCaches(rootNode);
   }

   private void updateCaches(BehaviorTreeNodeState node)
   {
      idToNodeMap.put(node.getID(), node);
      nodeNameList.add(node.getDefinition().getDescription());
      namesToNodesMap.put(node.getDefinition().getDescription(), node);
      nodesByID.add(node);

      for (BehaviorTreeNodeState child : node.getChildren())
      {
         updateCaches(child);
      }
   }

   public BehaviorTreeRootNode getRootNode()
   {
      return rootNode;
   }

   public MutableLong getNextID()
   {
      return nextID;
   }

   public Function<Class<?>, BehaviorTreeNodeState> getNewNodeSupplier()
   {
      return newNodeSupplier;
   }

   public TLongObjectMap<BehaviorTreeNodeState> getIDToNodeMap()
   {
      return idToNodeMap;
   }
}
