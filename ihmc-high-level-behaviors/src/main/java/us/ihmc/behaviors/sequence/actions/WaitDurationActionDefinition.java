package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;

public class WaitDurationActionDefinition implements BehaviorActionDefinition<WaitDurationActionDefinitionMessage>
{
   private String description = "Wait";
   private double waitDuration = 4.0;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("waitDuration", waitDuration);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
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

   @Override
   public void setDescription(String description)
   {
      this.description = description;
   }

   @Override
   public String getDescription()
   {
      return description;
   }
}
