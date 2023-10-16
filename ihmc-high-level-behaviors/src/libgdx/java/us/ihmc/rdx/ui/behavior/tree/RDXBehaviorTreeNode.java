package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateSupplier;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImStringWrapper;

public abstract class RDXBehaviorTreeNode implements BehaviorTreeNodeStateSupplier
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImStringWrapper descriptionWrapper;

   public void update()
   {
      if (descriptionWrapper == null)
      {
         descriptionWrapper = new ImStringWrapper(getDefinition()::getDescription,
                                                  getDefinition()::setDescription,
                                                  imString -> ImGuiTools.inputText(labels.getHidden("description"), imString));
      }
   }

   public ImStringWrapper getDescriptionWrapper()
   {
      return descriptionWrapper;
   }
}
