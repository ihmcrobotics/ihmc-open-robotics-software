package us.ihmc.rdx.ui.behavior.tree;

import com.fasterxml.jackson.databind.JsonNode;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.sequence.RDXAvailableBehaviorTreeFile;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

import javax.annotation.Nullable;

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

      LogTools.info("Loading {}", fileToLoad.getTreeFile().getFilesystemFile());
      RDXBaseUI.getInstance().getPrimary3DPanel().getNotificationManager().pushNotification("Loading %s".formatted(fileToLoad.getTreeFile().getFileName()));
      JSONFileTools.load(fileToLoad.getTreeFile(), jsonNode ->
      {
         loadedNode.setValue(loadFromFile(jsonNode, null, topologyOperationQueue, fileToLoad.getTreeFile().getFileName()));
      });

      return loadedNode.getValue();
   }

   private RDXBehaviorTreeNode<?, ?> loadFromFile(JsonNode jsonNode,
                                                  RDXBehaviorTreeNode<?, ?> parentNode,
                                                  BehaviorTreeTopologyOperationQueue topologyOperationQueue,
                                                  @Nullable String jsonFileName)
   {
      String typeName = jsonNode.get("type").textValue();

      RDXBehaviorTreeNode<?, ?> node = nodeBuilder.createNode(BehaviorTreeDefinitionRegistry.getClassFromTypeName(typeName),
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
            loadFromFile(childJsonNode, node, topologyOperationQueue, null);
         }
         else
         {
            WorkspaceResourceFile childFile = new WorkspaceResourceFile(behaviorTreeState.getSaveFileDirectory(), fileNode.asText());
            LogTools.info("Loading {}", childFile.getFilesystemFile());
            RDXBaseUI.getInstance().getPrimary3DPanel().getNotificationManager().pushNotification("Loading %s".formatted(childFile.getFileName()));
            JSONFileTools.load(childFile, childJSONNode ->
            {
               loadFromFile(childJSONNode, node, topologyOperationQueue, childFile.getFileName());
            });
         }
      });

      return node;
   }
}
