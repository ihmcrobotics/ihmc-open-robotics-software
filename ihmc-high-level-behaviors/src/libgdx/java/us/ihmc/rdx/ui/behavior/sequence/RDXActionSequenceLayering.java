package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

public class RDXActionSequenceLayering
{
   private final RDXActionSequence actionSequence;
   private final RecyclingArrayList<RDXActionSequenceLayeredNode> layeredNodes = new RecyclingArrayList<>(RDXActionSequenceLayeredNode::new);
   private int numberOfConcurrentActions = 0;
   private boolean previousWasExecuteWithNext = false;

   public RDXActionSequenceLayering(RDXActionSequence actionSequence)
   {
      this.actionSequence = actionSequence;
   }

   public void update()
   {
      layeredNodes.clear();
      numberOfConcurrentActions = 0;
      updateSubtree(actionSequence);
   }

   private void updateSubtree(RDXBehaviorTreeNode<?, ?> node)
   {
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         if (child instanceof RDXActionNode<?, ?> actionNode)
         {
            RDXActionSequenceLayeredNode layeredNode = layeredNodes.add();
            layeredNode.reset();

            boolean executeWithNextAction = actionNode.getState().getDefinition().getExecuteWithNextAction();

            if (!previousWasExecuteWithNext)
            {
               if (executeWithNextAction)
               {
                  numberOfConcurrentActions = countExecuteWithNext(actionSequence, actionNode.getState().getActionIndex(), 2);
               }
               else
               {
                  numberOfConcurrentActions = 1;
               }
            }

            layeredNode.setNumberOfConcurrentActions(numberOfConcurrentActions);
            previousWasExecuteWithNext = executeWithNextAction;

         }
         else
         {
            updateSubtree(child);
         }
      }
   }

   private int countExecuteWithNext(RDXBehaviorTreeNode<?, ?> node, int actionIndex, int count)
   {
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         if (child instanceof RDXActionNode<?, ?> actionNode)
         {
            if (actionNode.getState().getActionIndex() > actionIndex)
            {
               if (actionNode.getState().getDefinition().getExecuteWithNextAction())
               {
                  ++count;
               }
               else
               {
                  return count;
               }
            }
         }
         else
         {
            updateSubtree(child);
         }
      }

      return count;
   }

   public void renderPipelineIconForChild(int i)
   {
      ImGui.text(layeredNodes.get(i).getNumberOfConcurrentActions() + "");
      ImGui.sameLine();
   }
}
