package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SidedBodyPartPoseActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDefinition;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;

public class HandPoseActionDefinition implements BehaviorActionDefinition<SidedBodyPartPoseActionDefinitionMessage>, SidedObject
{
   private String description = "Hand pose";
   private RobotSide side = RobotSide.LEFT;
   private double trajectoryDuration = 4.0;
   private boolean executeWitNextAction = false;
   private boolean holdPoseInWorldLater = false;
   private boolean jointSpaceControl = true;
   private String palmParentFrameName;
   private final RigidBodyTransform palmTransformToParent = new RigidBodyTransform();

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", palmParentFrameName);
      JSONTools.toJSON(jsonNode, palmTransformToParent);
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("executeWithNextAction", executeWitNextAction);
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater);
      jsonNode.put("jointSpaceControl", jointSpaceControl);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      palmParentFrameName = jsonNode.get("parentFrame").textValue();
      JSONTools.toEuclid(jsonNode, palmTransformToParent);
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
      holdPoseInWorldLater = jsonNode.get("holdPoseInWorldLater").asBoolean();
      jointSpaceControl = jsonNode.get("jointSpaceControl").asBoolean();
   }

   @Override
   public void toMessage(SidedBodyPartPoseActionDefinitionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(palmParentFrameName);
      MessageTools.toMessage(palmTransformToParent, message.getTransformToParent());
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      message.setExecuteWithNextAction(executeWitNextAction);
      message.setHoldPoseInWorld(holdPoseInWorldLater);
      message.setJointSpaceControl(jointSpaceControl);
   }

   @Override
   public void fromMessage(SidedBodyPartPoseActionDefinitionMessage message)
   {
      palmParentFrameName = message.getParentFrame().getString(0);
      MessageTools.toEuclid(message.getTransformToParent(), palmTransformToParent);
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      executeWitNextAction = message.getExecuteWithNextAction();
      holdPoseInWorldLater = message.getHoldPoseInWorld();
      jointSpaceControl = message.getJointSpaceControl();
   }

   @Override
   public RobotSide getSide()
   {
      return side;
   }

   public void setSide(RobotSide side)
   {
      this.side = side;
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
   public boolean getExecuteWithNextAction()
   {
      return executeWitNextAction;
   }

   public void setExecuteWithNextAction(boolean executeWitNextAction)
   {
      this.executeWitNextAction = executeWitNextAction;
   }

   public boolean getHoldPoseInWorldLater()
   {
      return holdPoseInWorldLater;
   }

   public void setHoldPoseInWorldLater(boolean holdPoseInWorldLater)
   {
      this.holdPoseInWorldLater = holdPoseInWorldLater;
   }

   public boolean getJointSpaceControl()
   {
      return jointSpaceControl;
   }

   public void setJointSpaceControl(boolean jointSpaceControl)
   {
      this.jointSpaceControl = jointSpaceControl;
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

   public String getPalmParentFrameName()
   {
      return palmParentFrameName;
   }

   public void setPalmParentFrameName(String palmParentFrameName)
   {
      this.palmParentFrameName = palmParentFrameName;
   }

   public RigidBodyTransform getPalmTransformToParent()
   {
      return palmTransformToParent;
   }
}
