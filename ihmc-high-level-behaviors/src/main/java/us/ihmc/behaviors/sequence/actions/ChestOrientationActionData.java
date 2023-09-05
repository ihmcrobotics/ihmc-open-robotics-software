package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public class ChestOrientationActionData implements BehaviorActionData
{
   private String description = "Chest orientation";
   private final YawPitchRoll yawPitchRoll = new YawPitchRoll();
   private double trajectoryDuration = 4.0;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("yaw", yawPitchRoll.getYaw());
      jsonNode.put("pitch", yawPitchRoll.getPitch());
      jsonNode.put("roll", yawPitchRoll.getRoll());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      yawPitchRoll.setYaw(jsonNode.get("yaw").asDouble());
      yawPitchRoll.setPitch(jsonNode.get("pitch").asDouble());
      yawPitchRoll.setRoll(jsonNode.get("roll").asDouble());
   }

   public void toMessage(ChestOrientationActionMessage message)
   {
      message.setTrajectoryDuration(trajectoryDuration);
      message.getOrientation().setYaw(yawPitchRoll.getYaw());
      message.getOrientation().setPitch(yawPitchRoll.getPitch());
      message.getOrientation().setRoll(yawPitchRoll.getRoll());
   }

   public void fromMessage(ChestOrientationActionMessage message)
   {
      trajectoryDuration = message.getTrajectoryDuration();
      yawPitchRoll.setYaw(message.getOrientation().getYaw());
      yawPitchRoll.setPitch(message.getOrientation().getPitch());
      yawPitchRoll.setRoll(message.getOrientation().getRoll());
   }

   public YawPitchRoll getYawPitchRoll()
   {
      return yawPitchRoll;
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
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
