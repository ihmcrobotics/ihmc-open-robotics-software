package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.BodyPartPoseActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.behaviors.sequence.BehaviorActionSequenceTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.JSONTools;

import java.util.function.Consumer;

public class PelvisHeightActionData implements BehaviorActionData
{
   private String description = "Pelvis height";
   private double trajectoryDuration = 4.0;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ModifiableReferenceFrame pelvisInteractableReferenceFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());

   @Override
   public void setReferenceFrameLibrary(ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   @Override
   public void update()
   {
      BehaviorActionSequenceTools.accomodateFrameReplacement(pelvisInteractableReferenceFrame, referenceFrameLibrary);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", pelvisInteractableReferenceFrame.getReferenceFrame().getParent().getName());
      jsonNode.put("trajectoryDuration", trajectoryDuration);
      JSONTools.toJSON(jsonNode, pelvisInteractableReferenceFrame.getTransformToParent());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      trajectoryDuration = jsonNode.get("trajectoryDuration").asDouble();
      pelvisInteractableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(jsonNode.get("parentFrame").asText()).get());
      pelvisInteractableReferenceFrame.update(transformToParent -> JSONTools.toEuclid(jsonNode, transformToParent));
   }

   public void toMessage(BodyPartPoseActionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentFrame().getName());
      MessageTools.toMessage(pelvisInteractableReferenceFrame.getTransformToParent(), message.getTransformToParent());
      message.setTrajectoryDuration(trajectoryDuration);
   }

   public void fromMessage(BodyPartPoseActionMessage message)
   {
      pelvisInteractableReferenceFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(message.getParentFrame().getString(0)).get());
      pelvisInteractableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      trajectoryDuration = message.getTrajectoryDuration();
   }

   public double getHeight()
   {
      return getTransformToParent().getTranslationZ();
   }

   public void setHeight(double height)
   {
      getTransformToParent().getTranslation().set(getTransformToParent().getTranslationX(), getTransformToParent().getTranslationY(), height);
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   public ReferenceFrame getParentFrame()
   {
      return pelvisInteractableReferenceFrame.getReferenceFrame().getParent();
   }

   public ReferenceFrame getReferenceFrame()
   {
      return pelvisInteractableReferenceFrame.getReferenceFrame();
   }

   public void changeParentFrameWithoutMoving(ReferenceFrame parentFrame)
   {
      pelvisInteractableReferenceFrame.changeParentFrameWithoutMoving(parentFrame);
   }

   public void changeParentFrame(ReferenceFrame parentFrame)
   {
      pelvisInteractableReferenceFrame.changeParentFrame(parentFrame);
   }

   public void setTransformToParent(Consumer<RigidBodyTransform> transformToParentConsumer)
   {
      pelvisInteractableReferenceFrame.update(transformToParentConsumer);
   }

   public RigidBodyTransform getTransformToParent()
   {
      return pelvisInteractableReferenceFrame.getTransformToParent();
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
