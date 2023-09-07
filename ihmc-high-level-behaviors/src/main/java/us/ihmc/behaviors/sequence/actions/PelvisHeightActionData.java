package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.PelvisHeightActionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

import java.util.function.Consumer;

public class PelvisHeightActionData implements BehaviorActionData
{
   private String description = "Pelvis height";
   private double heightInWorld = 0.0;
   private double trajectoryDuration = 4.0;
   private final ModifiableReferenceFrame modifiableReferenceFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());

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

   public double getHeight()
   {
      return getTransformToParent().getTranslationZ();
   }

   public double getHeightInWorld()
   {
      return heightInWorld;
   }

   public void setHeight(double height)
   {
      getTransformToParent().setTranslationAndIdentityRotation(0, 0, height);
      this.heightInWorld = getReferenceFrame().getTransformToWorldFrame().getTranslationZ();
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
