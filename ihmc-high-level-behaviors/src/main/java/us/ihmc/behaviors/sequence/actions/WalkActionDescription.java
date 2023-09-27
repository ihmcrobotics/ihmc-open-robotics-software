package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionDescriptionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionDescription;
import us.ihmc.behaviors.sequence.BehaviorActionSequenceTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

import java.util.function.Consumer;

public class WalkActionDescription implements BehaviorActionDescription
{
   private String description = "Walk";
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ModifiableReferenceFrame goalFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private final SideDependentList<RigidBodyTransform> goalFootstepToParentTransforms = new SideDependentList<>(() -> new RigidBodyTransform());
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;

   @Override
   public void setReferenceFrameLibrary(ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   @Override
   public void update()
   {
      BehaviorActionSequenceTools.accommodateFrameReplacement(goalFrame, referenceFrameLibrary);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", goalFrame.getReferenceFrame().getParent().getName());
      JSONTools.toJSON(jsonNode, goalFrame.getTransformToParent());
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
      goalFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(jsonNode.get("parentFrame").asText()));
      goalFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
      for (RobotSide side : RobotSide.values)
      {
         JsonNode goalFootNode = jsonNode.get(side.getCamelCaseName() + "GoalFootTransform");
         JSONTools.toEuclid(goalFootNode, goalFootstepToParentTransforms.get(side));
      }
      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();
   }

   public void toMessage(WalkActionDescriptionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentFrame().getName());
      MessageTools.toMessage(goalFrame.getTransformToParent(), message.getTransformToParent());
      MessageTools.toMessage(goalFootstepToParentTransforms.get(RobotSide.LEFT), message.getLeftGoalFootTransformToGizmo());
      MessageTools.toMessage(goalFootstepToParentTransforms.get(RobotSide.RIGHT), message.getRightGoalFootTransformToGizmo());
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
   }

   public void fromMessage(WalkActionDescriptionMessage message)
   {
      goalFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(message.getParentFrame().getString(0)));
      goalFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      MessageTools.toEuclid(message.getLeftGoalFootTransformToGizmo(), goalFootstepToParentTransforms.get(RobotSide.LEFT));
      MessageTools.toEuclid(message.getRightGoalFootTransformToGizmo(), goalFootstepToParentTransforms.get(RobotSide.RIGHT));
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();
   }

   public ReferenceFrame getParentFrame()
   {
      return goalFrame.getReferenceFrame().getParent();
   }

   public ReferenceFrame getGoalFrame()
   {
      return goalFrame.getReferenceFrame();
   }

   public void changeParentFrameWithoutMoving(ReferenceFrame parentFrame)
   {
      goalFrame.changeParentFrameWithoutMoving(parentFrame);
   }

   public void changeParentFrame(ReferenceFrame parentFrame)
   {
      goalFrame.changeParentFrame(parentFrame);
   }

   public void setTransformToParent(Consumer<RigidBodyTransform> transformToParentConsumer)
   {
      goalFrame.update(transformToParentConsumer);
   }

   public RigidBodyTransform getTransformToParent()
   {
      return goalFrame.getTransformToParent();
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
