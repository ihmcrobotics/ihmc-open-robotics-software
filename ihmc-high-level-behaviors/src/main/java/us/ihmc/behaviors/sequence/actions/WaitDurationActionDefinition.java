package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;

public class WaitDurationActionDefinition extends BehaviorActionDefinition<WaitDurationActionDefinitionMessage>
{
   private double waitDuration = 4.0;

   public WaitDurationActionDefinition()
   {
      super("Wait");
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("waitDuration", waitDuration);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      waitDuration = jsonNode.get("waitDuration").asDouble();
   }

   @Override
   public void toMessage(WaitDurationActionDefinitionMessage message)
   {
      message.setWaitDuration(waitDuration);
   }

   @Override
   public void fromMessage(WaitDurationActionDefinitionMessage message)
   {
      waitDuration = message.getWaitDuration();
   }

   public double getWaitDuration()
   {
      return waitDuration;
   }

   public void setWaitDuration(double waitDuration)
   {
      this.waitDuration = waitDuration;
   }
}
