package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WaitDurationActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class WaitDurationActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalDouble waitDuration;

   // On disk fields
   private double onDiskWaitDuration;

   public WaitDurationActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      waitDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, this, 4.0);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("waitDuration", waitDuration.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      waitDuration.setValue(jsonNode.get("waitDuration").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskWaitDuration = waitDuration.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      waitDuration.setValue(onDiskWaitDuration);
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= waitDuration.getValue() == onDiskWaitDuration;

      return !unchanged;
   }

   public void toMessage(WaitDurationActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setWaitDuration(waitDuration.toMessage());
   }

   public void fromMessage(WaitDurationActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      waitDuration.fromMessage(message.getWaitDuration());
   }

   public double getWaitDuration()
   {
      return waitDuration.getValue();
   }

   public void setWaitDuration(double waitDuration)
   {
      this.waitDuration.setValue(waitDuration);
   }
}
