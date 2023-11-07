package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;

public class WaitDurationActionDefinition extends ActionNodeDefinition
{
   private double waitDuration = 4.0;

   public WaitDurationActionDefinition(CRDTInfo crdtInfo)
   {
      super(crdtInfo);
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

   public void toMessage(WaitDurationActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setWaitDuration(waitDuration);
   }

   public void fromMessage(WaitDurationActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

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
