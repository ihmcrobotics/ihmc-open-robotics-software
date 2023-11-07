package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;

public class HandPoseActionDefinition extends ActionNodeDefinition implements SidedObject
{
   private RobotSide side = RobotSide.LEFT;
   private double trajectoryDuration = 4.0;
   private boolean holdPoseInWorldLater = false;
   private boolean jointSpaceControl = true;
   private String palmParentFrameName;
   private final RigidBodyTransform palmTransformToParent = new RigidBodyTransform();

   public HandPoseActionDefinition(CRDTInfo crdtInfo)
   {
      super(crdtInfo);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("parentFrame", palmParentFrameName);
      JSONTools.toJSON(jsonNode, palmTransformToParent);
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater);
      jsonNode.put("jointSpaceControl", jointSpaceControl);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      palmParentFrameName = jsonNode.get("parentFrame").textValue();
      JSONTools.toEuclid(jsonNode, palmTransformToParent);
      holdPoseInWorldLater = jsonNode.get("holdPoseInWorldLater").asBoolean();
      jointSpaceControl = jsonNode.get("jointSpaceControl").asBoolean();
   }

   public void toMessage(HandPoseActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setParentFrameName(palmParentFrameName);
      MessageTools.toMessage(palmTransformToParent, message.getTransformToParent());
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      message.setExecuteWithNextAction(getExecuteWithNextAction());
      message.setHoldPoseInWorld(holdPoseInWorldLater);
      message.setJointSpaceControl(jointSpaceControl);
   }

   public void fromMessage(HandPoseActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      palmParentFrameName = message.getParentFrameNameAsString();
      MessageTools.toEuclid(message.getTransformToParent(), palmTransformToParent);
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      setExecuteWithNextAction(message.getExecuteWithNextAction());
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
