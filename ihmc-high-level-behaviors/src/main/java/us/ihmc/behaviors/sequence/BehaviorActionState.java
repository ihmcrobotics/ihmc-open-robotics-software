package us.ihmc.behaviors.sequence;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.communication.packets.Packet;

// TODO: Include toMessage and fromMessage
public abstract class BehaviorActionState<T extends Packet<T>> implements BehaviorActionDefinitionSupplier<T>
{
   /** The action's unique ID. */
   private final long id;
   private int actionIndex = -1;
   private boolean isNextForExecution = false;
   private boolean isToBeExecutedConcurrently = false;

   public BehaviorActionState()
   {
      // TODO: Make parameter
      id = BehaviorActionSequence.NEXT_ID.getAndIncrement();
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

   public void toMessage(T message)
   {
      getDefinition().toMessage(message);
      // TODO: Pack
   }

   public void fromMessage(T message)
   {
      getDefinition().fromMessage(message);
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
