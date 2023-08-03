package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.robotics.robotSide.RobotSide;

public class ArmJointAnglesActionData implements BehaviorActionData
{
   public static final int NUMBER_OF_JOINTS = 7;

   private String description = "Arm joint angles";
   private final double[] jointAngles = new double[NUMBER_OF_JOINTS];
   private RobotSide side = RobotSide.LEFT;
   private double trajectoryDuration = 4.0;

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      for (int i = 0; i < NUMBER_OF_JOINTS; i++)
      {
         jsonNode.put("j" + i, jointAngles[i]);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      for (int i = 0; i < NUMBER_OF_JOINTS; i++)
      {
         jointAngles[i] = jsonNode.get("j" + i).asDouble();
      }
   }

   public void toMessage(ArmJointAnglesActionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      for (int i = 0; i < NUMBER_OF_JOINTS; i++)
      {
         message.getJointAngles()[i] = jointAngles[i];
      }
   }

   public void fromMessage(ArmJointAnglesActionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      for (int i = 0; i < NUMBER_OF_JOINTS; i++)
      {
         jointAngles[i] = message.getJointAngles()[i];
      }
   }

   public double[] getJointAngles()
   {
      return jointAngles;
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public RobotSide getSide()
   {
      return side;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
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
