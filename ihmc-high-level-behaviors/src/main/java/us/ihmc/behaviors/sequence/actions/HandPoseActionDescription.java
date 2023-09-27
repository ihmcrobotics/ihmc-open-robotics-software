package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SidedBodyPartPoseActionDescriptionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.FrameBasedBehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

public class HandPoseActionDescription extends FrameBasedBehaviorActionData
{
   private String description = "Hand pose";
   private RobotSide side = RobotSide.LEFT;
   private double trajectoryDuration = 4.0;
   private boolean executeWitNextAction = false;
   private boolean holdPoseInWorldLater = false;
   private boolean jointSpaceControl = true;

   @Override
   public String getDescription()
   {
      return description;
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", palmFrame.getReferenceFrame().getParent().getName());
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      JSONTools.toJSON(jsonNode, palmFrame.getTransformToParent());
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
      palmFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(jsonNode.get("parentFrame").asText()));
      palmFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
      holdPoseInWorldLater = jsonNode.get("holdPoseInWorldLater").asBoolean();
      jointSpaceControl = jsonNode.get("jointSpaceControl").asBoolean();
   }

   public void toMessage(SidedBodyPartPoseActionDescriptionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentFrame().getName());
      MessageTools.toMessage(palmFrame.getTransformToParent(), message.getTransformToParent());
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      message.setExecuteWithNextAction(executeWitNextAction);
      message.setHoldPoseInWorld(holdPoseInWorldLater);
      message.setJointSpaceControl(jointSpaceControl);
   }

   public void fromMessage(SidedBodyPartPoseActionDescriptionMessage message)
   {
      palmFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(message.getParentFrame().getString(0)));
      palmFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      executeWitNextAction = message.getExecuteWithNextAction();
      holdPoseInWorldLater = message.getHoldPoseInWorld();
      jointSpaceControl = message.getJointSpaceControl();
   }

   public ReferenceFrame getParentFrame()
   {
      return palmFrame.getReferenceFrame().getParent();
   }

   public ReferenceFrame getPalmFrame()
   {
      return palmFrame.getReferenceFrame();
   }

   public void changeParentFrameWithoutMoving(ReferenceFrame parentFrame)
   {
      palmFrame.changeParentFrameWithoutMoving(parentFrame);
   }

   public void changeParentFrame(ReferenceFrame parentFrame)
   {
      palmFrame.changeParentFrame(parentFrame);
   }

   public void setTransformToParent(Consumer<RigidBodyTransform> transformToParentConsumer)
   {
      palmFrame.update(transformToParentConsumer);
   }

   public RigidBodyTransform getTransformToParent()
   {
      return palmFrame.getTransformToParent();
   }

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
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("side", side.getLowerCaseName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      jsonNode.put("parentFrame", getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      JSONTools.toJSON(jsonNode, getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      getConditionalReferenceFrame().setParentFrameName(jsonNode.get("parentFrame").textValue());
      JSONTools.toEuclid(jsonNode, getTransformToParent());
   }

   public void toMessage(SidedBodyPartPoseActionDescriptionMessage message)
   {
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      MessageTools.toMessage(getTransformToParent(), message.getTransformToParent());
   }

   public void fromMessage(SidedBodyPartPoseActionDescriptionMessage message)
   {
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      getConditionalReferenceFrame().setParentFrameName(message.getParentFrame().getString(0));
      MessageTools.toEuclid(message.getTransformToParent(), getTransformToParent());
   }

   public ReferenceFrameLibrary getReferenceFrameLibrary()
   {
      return referenceFrameLibrary;
   }
}
