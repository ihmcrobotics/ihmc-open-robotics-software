package us.ihmc.rdx.ui.behavior.tree;

import com.fasterxml.jackson.databind.JsonNode;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeExtensionAddAndFreeze;
import us.ihmc.rdx.ui.behavior.sequence.RDXAvailableBehaviorTreeFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;

public class RDXBehaviorTreeFileLoader
{
   private final BehaviorTreeState behaviorTreeState;
   private final RDXBehaviorTreeNodeBuilder nodeBuilder;

   public RDXBehaviorTreeFileLoader(BehaviorTreeState behaviorTreeState,
                                    RDXBehaviorTreeNodeBuilder nodeBuilder)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.nodeBuilder = nodeBuilder;
   }

   public BehaviorTreeNodeExtension<?, ?, ?, ?> loadFromFile(RDXAvailableBehaviorTreeFile fileToLoad, BehaviorTreeModificationQueue modificationQueue)
   {
      MutableObject<RDXBehaviorTreeNode<?, ?>> loadedNode = new MutableObject<>();

      JSONFileTools.load(fileToLoad.getTreeFile(), jsonNode ->
      {
         loadedNode.setValue(loadFromFile(jsonNode, null, modificationQueue));
      });

      return loadedNode.getValue();
   }

   private RDXBehaviorTreeNode<?, ?> loadFromFile(JsonNode jsonNode, RDXBehaviorTreeNode<?, ?> parentNode, BehaviorTreeModificationQueue modificationQueue)
   {
      String typeName = jsonNode.get("type").textValue();

      RDXBehaviorTreeNode<?, ?> node = nodeBuilder.createNode(BehaviorTreeDefinitionRegistry.getClassFromTypeName(typeName),
                                                              behaviorTreeState.getNextID().getAndIncrement());

      node.getDefinition().loadFromFile(jsonNode);

      if (parentNode != null)
      {
         modificationQueue.accept(new BehaviorTreeNodeExtensionAddAndFreeze<>(node, parentNode));
      }

      JSONTools.forEachArrayElement(jsonNode, "children", childJsonNode ->
      {
         loadFromFile(childJsonNode, node, modificationQueue);
      });

      return node;
   }
}
