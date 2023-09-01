package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PelvisHeightActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;

public class PelvisHeightActionData implements BehaviorActionData
{
   private String description = "Pelvis height";
   private double heightInWorld = 0.0;
   private double trajectoryDuration = 1000.0;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("heightInWorld", heightInWorld);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      heightInWorld = jsonNode.get("heightInWorld").asDouble();
   }

   public void toMessage(PelvisHeightActionMessage message)
   {
      message.setTrajectoryDuration(trajectoryDuration);
      message.setHeightInWorld(heightInWorld);
   }

   public void fromMessage(PelvisHeightActionMessage message)
   {
      trajectoryDuration = message.getTrajectoryDuration();
      heightInWorld = message.getHeightInWorld();
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   public double getHeightInWorld()
   {
      return heightInWorld;
   }

   public void setHeightInWorld(double heightInWorld)
   {
      this.heightInWorld = heightInWorld;
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
