package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;

public class WaitDurationActionData implements BehaviorActionData
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

   public void toMessage(WaitDurationActionMessage message)
   {
      message.setWaitDuration(waitDuration);
   }

   public void fromMessage(WaitDurationActionMessage message)
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
