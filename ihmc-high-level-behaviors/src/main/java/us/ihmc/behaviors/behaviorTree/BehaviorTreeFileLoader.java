package us.ihmc.behaviors.behaviorTree;

import com.fasterxml.jackson.databind.JsonNode;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

public class BehaviorTreeFileLoader<T extends BehaviorTreeNodeLayer<T, ?, ?, ?>>
{
   private final BehaviorTreeState behaviorTreeState;
   private final BehaviorTreeNodeStateBuilder<T> nodeBuilder;

   public BehaviorTreeFileLoader(BehaviorTreeState behaviorTreeState, BehaviorTreeNodeStateBuilder<T> nodeBuilder)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.nodeBuilder = nodeBuilder;
   }

   public T loadFromFile(WorkspaceResourceFile fileToLoad, BehaviorTreeTopologyOperationQueue topologyOperationQueue)
   {
      MutableObject<T> loadedNode = new MutableObject<>();

      LogTools.info("Loading {}", fileToLoad.getFilesystemFile());
      JSONFileTools.load(fileToLoad, jsonNode ->
      {
         loadedNode.setValue(loadFromFile(jsonNode, null, topologyOperationQueue));
      });

      return loadedNode.getValue();
   }

   private T loadFromFile(JsonNode jsonNode,
                                                       T parentNode,
                                                       BehaviorTreeTopologyOperationQueue topologyOperationQueue)
   {
      String typeName = jsonNode.get("type").textValue();

      T node = nodeBuilder.createNode(BehaviorTreeDefinitionRegistry.getClassFromTypeName(typeName),
                                      behaviorTreeState.getAndIncrementNextID(),
                                      behaviorTreeState.getCRDTInfo(),
                                      behaviorTreeState.getSaveFileDirectory());
      node.getDefinition().loadFromFile(jsonNode);
      LogTools.info("Creating node: {}:{}", node.getDefinition().getName(), node.getState().getID());

      if (parentNode != null)
      {
         topologyOperationQueue.queueAddAndFreezeNode(node, parentNode);
      }

      JSONTools.forEachArrayElement(jsonNode, "children", childJsonNode ->
      {
         JsonNode fileNode = childJsonNode.get("file");
         if (fileNode == null)
         {
            loadFromFile(childJsonNode, node, topologyOperationQueue);
         }
         else
         {
            WorkspaceResourceFile childFile = new WorkspaceResourceFile(behaviorTreeState.getSaveFileDirectory(), fileNode.asText());
            LogTools.info("Loading {}", childFile.getFilesystemFile());
            JSONFileTools.load(childFile, childJSONNode ->
            {
               loadFromFile(childJSONNode, node, topologyOperationQueue);
            });
         }
      });

      // Set the on disk fields after the children are added so we
      // can mark modified when children topology changes
      topologyOperationQueue.queueOperation(() -> node.getDefinition().setOnDiskFields());

      return node;
   }
}
