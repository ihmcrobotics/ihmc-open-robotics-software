package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

import java.util.function.Consumer;

public class WalkActionData implements BehaviorActionData
{
   private String description = "Walk";
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ModifiableReferenceFrame modifiableReferenceFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private final SideDependentList<RigidBodyTransform> goalFootstepToParentTransforms = new SideDependentList<>(() -> new RigidBodyTransform());
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;

   @Override
   public void setReferenceFrameLibrary(ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", modifiableReferenceFrame.getReferenceFrame().getParent().getName());
      JSONTools.toJSON(jsonNode, modifiableReferenceFrame.getTransformToParent());
      for (RobotSide side : RobotSide.values)
      {
         ObjectNode goalFootNode = jsonNode.putObject(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toJSON(goalFootNode, goalFootstepToParentTransforms.get(side));
      }
      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      modifiableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(jsonNode.get("parentFrame").asText()).get());
      modifiableReferenceFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
      for (RobotSide side : RobotSide.values)
      {
         JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toEuclid(goalFootNode, goalFootstepToParentTransforms.get(side));
      }
      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();
   }

   public void toMessage(WalkActionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentReferenceFrame().getName());
      MessageTools.toMessage(modifiableReferenceFrame.getTransformToParent(), message.getTransformToParent());
      MessageTools.toMessage(goalFootstepToParentTransforms.get(RobotSide.LEFT), message.getLeftGoalFootTransformToGizmo());
      MessageTools.toMessage(goalFootstepToParentTransforms.get(RobotSide.RIGHT), message.getRightGoalFootTransformToGizmo());
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
   }

   public void fromMessage(WalkActionMessage message)
   {
      modifiableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByName(message.getParentFrame().getString(0)).get());
      modifiableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      MessageTools.toEuclid(message.getLeftGoalFootTransformToGizmo(), goalFootstepToParentTransforms.get(RobotSide.LEFT));
      MessageTools.toEuclid(message.getRightGoalFootTransformToGizmo(), goalFootstepToParentTransforms.get(RobotSide.RIGHT));
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();
   }

   public ReferenceFrame getParentReferenceFrame()
   {
      return modifiableReferenceFrame.getReferenceFrame().getParent();
   }

   public ReferenceFrame getReferenceFrame()
   {
      return modifiableReferenceFrame.getReferenceFrame();
   }

   public void changeParentFrameWithoutMoving(ReferenceFrame parentFrame)
   {
      modifiableReferenceFrame.changeParentFrameWithoutMoving(parentFrame);
   }

   public void changeParentFrame(ReferenceFrame parentFrame)
   {
      modifiableReferenceFrame.changeParentFrame(parentFrame);
   }

   public void setTransformToParent(Consumer<RigidBodyTransform> transformToParentConsumer)
   {
      modifiableReferenceFrame.update(transformToParentConsumer);
   }

   public RigidBodyTransform getTransformToParent()
   {
      return modifiableReferenceFrame.getTransformToParent();
   }

   public SideDependentList<RigidBodyTransform> getGoalFootstepToParentTransforms()
   {
      return goalFootstepToParentTransforms;
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public double getTransferDuration()
   {
      return transferDuration;
   }

   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration = transferDuration;
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
