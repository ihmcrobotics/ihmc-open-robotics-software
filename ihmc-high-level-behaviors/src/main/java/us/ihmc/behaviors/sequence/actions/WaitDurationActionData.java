package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;

public class WaitDurationActionData implements BehaviorActionData
{
   private double waitDuration = 4.0;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("waitDuration", waitDuration);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
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
}
