package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.NeckActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.tools.io.JSONTools;

public class NeckActionDefinition extends ActionNodeDefinition
{
   private final CRDTBidirectionalDouble pitch;
   private final CRDTBidirectionalDouble yaw;
   private final CRDTBidirectionalDouble trajectoryDuration;

   // On disk fields
   private double onDiskPitch;
   private double onDiskYaw;
   private double onDiskTrajectoryDuration;

   public NeckActionDefinition(BehaviorTreeRootNodeDefinition rootNode)
   {
      super(rootNode);

      pitch = new CRDTBidirectionalDouble(this, 0.0);
      yaw = new CRDTBidirectionalDouble(this, 0.0);
      trajectoryDuration = new CRDTBidirectionalDouble(this, 4.0);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("pitchInDegrees", JSONTools.toJsonRadians(pitch.getValue()));
      jsonNode.put("yawInDegrees", JSONTools.toJsonRadians(yaw.getValue()));
      jsonNode.put("trajectoryDuration", trajectoryDuration.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      pitch.setValue(Math.toRadians(jsonNode.get("pitchInDegrees").asDouble()));
      yaw.setValue(Math.toRadians(jsonNode.get("yawInDegrees").asDouble()));
      trajectoryDuration.setValue(jsonNode.get("trajectoryDuration").asDouble());
   }

   @Override
   public void setOnDiskFields()
   {
      super.setOnDiskFields();

      onDiskPitch = pitch.getValue();
      onDiskYaw = yaw.getValue();
      onDiskTrajectoryDuration = trajectoryDuration.getValue();
   }

   @Override
   public void undoAllNontopologicalChanges()
   {
      super.undoAllNontopologicalChanges();

      if (isUndoAvailable())
      {
         pitch.setValue(onDiskPitch);
         yaw.setValue(onDiskYaw);
         trajectoryDuration.setValue(onDiskTrajectoryDuration);
      }
   }

   @Override
   public boolean hasChanges()
   {
      boolean unchanged = !super.hasChanges();

      unchanged &= pitch.getValue() == onDiskPitch;
      unchanged &= yaw.getValue() == onDiskYaw;
      unchanged &= trajectoryDuration.getValue() == onDiskTrajectoryDuration;

      return !unchanged;
   }

   public void toMessage(NeckActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setPitch(pitch.toMessage());
      message.setYaw(yaw.toMessage());
      message.setTrajectoryDuration(trajectoryDuration.toMessage());
   }

   public void fromMessage(NeckActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      pitch.fromMessage(message.getPitch());
      yaw.fromMessage(message.getYaw());
      trajectoryDuration.fromMessage(message.getTrajectoryDuration());
   }

   public double getPitch()
   {
      return pitch.getValue();
   }

   public void setPitch(double pitch)
   {
      this.pitch.setValue(pitch);
   }

   public double getYaw()
   {
      return yaw.getValue();
   }

   public void setYaw(double yaw)
   {
      this.yaw.setValue(yaw);
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration.getValue();
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration.setValue(trajectoryDuration);
   }
}
