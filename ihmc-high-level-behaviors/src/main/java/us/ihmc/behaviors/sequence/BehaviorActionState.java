package us.ihmc.behaviors.sequence;

import behavior_msgs.msg.dds.BehaviorActionStateMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

public abstract class BehaviorActionState implements BehaviorActionDefinitionSupplier
{
   /** The action's unique ID. */
   private long id;
   private int actionIndex = -1;
   private boolean isNextForExecution = false;
   private boolean isToBeExecutedConcurrently = false;

   public BehaviorActionState()
   {
      // TODO: Re-enable when we do the CRDT
//      id = BehaviorActionSequence.NEXT_ID.getAndIncrement();
   }

   public void update()
   {

   }

   /** This is the default, if there is no subtree for this action */
   public void saveToFile(ObjectNode jsonNode)
   {
      getDefinition().saveToFile(jsonNode);
   }

   /** This is the default, if there is no subtree for this action */
   public void loadFromFile(JsonNode jsonNode)
   {
      getDefinition().loadFromFile(jsonNode);
      update();
      // TODO: Pack
   }

   public void toMessage(BehaviorActionStateMessage message)
   {
      message.setId(id);
      message.setActionIndex(actionIndex);
      message.setIsNextForExecution(isNextForExecution);
      message.setIsToBeExecutedConcurrently(isToBeExecutedConcurrently);
   }

   public void fromMessage(BehaviorActionStateMessage message)
   {
//      if (id != message.getId())
//         LogTools.error("IDs should match!");

      id = message.getId();
      actionIndex = message.getActionIndex();
      isNextForExecution = message.getIsNextForExecution();
      isToBeExecutedConcurrently = message.getIsToBeExecutedConcurrently();
   }

   /** The action's unique ID. */
   public long getID()
   {
      return id;
   }

   public void setActionIndex(int actionIndex)
   {
      this.actionIndex = actionIndex;
   }

   public int getActionIndex()
   {
      return actionIndex;
   }

   public void setIsNextForExecution(boolean isNextForExecution)
   {
      this.isNextForExecution = isNextForExecution;
   }

   public boolean getIsNextForExecution()
   {
      return isNextForExecution;
   }

   public void setIsToBeExecutedConcurrently(boolean isToBeExecutedConcurrently)
   {
      this.isToBeExecutedConcurrently = isToBeExecutedConcurrently;
   }

   public boolean getIsToBeExecutedConcurrently()
   {
      return isToBeExecutedConcurrently;
   }
}
