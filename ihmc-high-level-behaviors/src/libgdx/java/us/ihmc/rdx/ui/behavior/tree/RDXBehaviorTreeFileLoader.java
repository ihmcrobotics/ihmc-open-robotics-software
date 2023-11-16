package us.ihmc.rdx.ui.behavior.tree;

import com.fasterxml.jackson.databind.JsonNode;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.behavior.sequence.RDXAvailableBehaviorTreeFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;

public class RDXBehaviorTreeFileLoader
{
   private final BehaviorTreeState behaviorTreeState;
   private final RDXBehaviorTreeNodeBuilder nodeBuilder;

   public RDXBehaviorTreeFileLoader(BehaviorTreeState behaviorTreeState, RDXBehaviorTreeNodeBuilder nodeBuilder)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.nodeBuilder = nodeBuilder;
   }

   public RDXBehaviorTreeNode<?, ?> loadFromFile(RDXAvailableBehaviorTreeFile fileToLoad, BehaviorTreeTopologyOperationQueue topologyOperationQueue)
   {
      MutableObject<RDXBehaviorTreeNode<?, ?>> loadedNode = new MutableObject<>();

      JSONFileTools.load(fileToLoad.getTreeFile(), jsonNode ->
      {
         loadedNode.setValue(loadFromFile(jsonNode, null, topologyOperationQueue));
      });

      loadedNode.getValue().getDefinition().setJSONFileName(fileToLoad.getTreeFile().getFileName());

      return loadedNode.getValue();
   }

   private RDXBehaviorTreeNode<?, ?> loadFromFile(JsonNode jsonNode,
                                                  RDXBehaviorTreeNode<?, ?> parentNode,
                                                  BehaviorTreeTopologyOperationQueue topologyOperationQueue)
   {
      String typeName = jsonNode.get("type").textValue();

      RDXBehaviorTreeNode<?, ?> node = nodeBuilder.createNode(BehaviorTreeDefinitionRegistry.getClassFromTypeName(typeName),
                                                              behaviorTreeState.getAndIncrementNextID(),
                                                              behaviorTreeState.getCRDTInfo(),
                                                              behaviorTreeState.getSaveFileDirectory());
      node.getDefinition().loadFromFile(jsonNode);
      LogTools.info("Creating node: {}:{}", node.getDefinition().getDescription(), node.getState().getID());

      if (parentNode != null)
      {
         topologyOperationQueue.queueAddAndFreezeNode(node, parentNode);
      }

      JSONTools.forEachArrayElement(jsonNode, "children", childJsonNode ->
      {
         loadFromFile(childJsonNode, node, topologyOperationQueue);
      });

      return node;
   }
}
