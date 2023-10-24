package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImStringWrapper;

import java.util.ArrayList;
import java.util.List;

public abstract class RDXBehaviorTreeNode<S extends BehaviorTreeNodeState<D>,
                                          D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNodeExtension<RDXBehaviorTreeNode<?, ?>, S, S, D>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImStringWrapper descriptionWrapper;

   private final List<RDXBehaviorTreeNode<?, ?>> children = new ArrayList<>();

   public void update()
   {
      if (descriptionWrapper == null)
      {
         descriptionWrapper = new ImStringWrapper(getDefinition()::getDescription,
                                                  getDefinition()::setDescription,
                                                  imString -> ImGuiTools.inputText(labels.getHidden("description"), imString));
      }
   }

   @Override
   public void destroy()
   {

   }

   public ImStringWrapper getDescriptionWrapper()
   {
      return descriptionWrapper;
   }

   public List<RDXBehaviorTreeNode<?, ?>> getChildren()
   {
      return children;
   }

   @Override
   public S getExtendedNode()
   {
      return getState();
   }
}
