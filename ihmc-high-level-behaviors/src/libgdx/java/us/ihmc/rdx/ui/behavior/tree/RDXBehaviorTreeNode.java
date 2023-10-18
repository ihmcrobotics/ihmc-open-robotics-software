package us.ihmc.rdx.ui.behavior.tree;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateSupplier;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImStringWrapper;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeModificationQueue;
import us.ihmc.rdx.ui.behavior.tree.modification.RDXBehaviorTreeNodeReplacement;

import java.util.ArrayList;
import java.util.List;

public abstract class RDXBehaviorTreeNode implements BehaviorTreeNodeStateSupplier
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImStringWrapper descriptionWrapper;

   private final List<RDXBehaviorTreeNode> children = new ArrayList<>();

   public void loadFromFile(JsonNode jsonNode,
                            RDXBehaviorTreeNode parentNode,
                            RDXBehaviorTreeModificationQueue modificationQueue)
   {
      if (parentNode != null)
      {

         modificationQueue.accept(new RDXBehaviorTreeNodeReplacement(this, parentNode));
      }
   }

   public void update()
   {
      if (descriptionWrapper == null)
      {
         descriptionWrapper = new ImStringWrapper(getDefinition()::getDescription,
                                                  getDefinition()::setDescription,
                                                  imString -> ImGuiTools.inputText(labels.getHidden("description"), imString));
      }
   }

   public void destroy()
   {

   }

   public ImStringWrapper getDescriptionWrapper()
   {
      return descriptionWrapper;
   }

   public List<RDXBehaviorTreeNode> getChildren()
   {
      return children;
   }
}
