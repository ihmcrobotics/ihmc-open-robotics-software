package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SidedBodyPartPoseActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.behaviors.sequence.BehaviorActionSequenceTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.JSONTools;

import java.util.function.Consumer;

public class HandPoseActionData implements BehaviorActionData
{
   private String description = "Hand pose";
   private RobotSide side = RobotSide.LEFT;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ModifiableReferenceFrame palmFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private double trajectoryDuration = 4.0;
   private boolean executeWitNextAction = false;
   private boolean holdPoseInWorldLater = false;

   @Override
   public void setReferenceFrameLibrary(ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   @Override
   public void update()
   {
      BehaviorActionSequenceTools.accomodateFrameReplacement(palmFrame, referenceFrameLibrary);
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
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      side = RobotSide.getSideFromString(jsonNode.get("side").asText());
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      palmFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(jsonNode.get("parentFrame").asText()).get());
      palmFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
      executeWitNextAction = jsonNode.get("executeWithNextAction").asBoolean();
      holdPoseInWorldLater = jsonNode.get("holdPoseInWorldLater").asBoolean();
   }

   public void toMessage(SidedBodyPartPoseActionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentFrame().getName());
      MessageTools.toMessage(palmFrame.getTransformToParent(), message.getTransformToParent());
      message.setRobotSide(side.toByte());
      message.setTrajectoryDuration(trajectoryDuration);
      message.setExecuteWithNextAction(executeWitNextAction);
      message.setHoldPoseInWorld(holdPoseInWorldLater);
   }

   public void fromMessage(SidedBodyPartPoseActionMessage message)
   {
      palmFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(message.getParentFrame().getString(0)).get());
      palmFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      side = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      executeWitNextAction = message.getExecuteWithNextAction();
      holdPoseInWorldLater = message.getHoldPoseInWorld();
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
