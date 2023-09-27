package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepActionDescriptionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionDescriptionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.FrameBasedBehaviorActionData;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.tools.io.JSONTools;

public class FootstepPlanActionDescription extends FrameBasedBehaviorActionData
{
   private String description = "Footstep plan";
   private double swingDuration = 1.2;
   private double transferDuration = 0.8;
   private final RecyclingArrayList<FootstepActionData> footsteps = new RecyclingArrayList<>(FootstepActionData::new);

   @Override
   public String getDescription()
   {
      return description;
   }

   @Override
   public void setDescription(String description)
   {
      this.description = description;
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("parentFrame", planFrame.getReferenceFrame().getParent().getName());
      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);

      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (FootstepActionData footstep : footsteps)
      {
         ObjectNode footstepNode = foostepsArrayNode.addObject();
         footstep.saveToFile(footstepNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      planFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(jsonNode.get("parentFrame").asText()));
      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();

      footsteps.clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.add().loadFromFile(footstepNode));
   }

   public void fromMessage(FootstepPlanActionDescriptionMessage message)
   {
      planFrame.changeParentFrame(referenceFrameLibrary.findFrameByNameOrWorld(message.getParentFrame().getString(0)));
      planFrame.update(transformToParent -> MessageTools.toEuclid(message.getTransformToParent(), transformToParent));
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();

      footsteps.clear();
      for (FootstepActionDescriptionMessage footstep : message.getFootsteps())
      {
         footsteps.add().fromMessage(footstep);
      }
   }

   public ReferenceFrame getParentFrame()
   {
      return planFrame.getReferenceFrame().getParent();
   }

   public ReferenceFrame getPlanFrame()
   {
      return planFrame.getReferenceFrame();
   }

   public void changeParentFrameWithoutMoving(ReferenceFrame parentFrame)
   {
      planFrame.changeParentFrameWithoutMoving(parentFrame);
   }

   public void changeParentFrame(ReferenceFrame parentFrame)
   {
      planFrame.changeParentFrame(parentFrame);
   }

   public void setTransformToParent(Consumer<RigidBodyTransform> transformToParentConsumer)
   {
      planFrame.update(transformToParentConsumer);
   }

   public RigidBodyTransform getTransformToParent()
   {
      return planFrame.getTransformToParent();
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

   public RecyclingArrayList<FootstepActionData> getFootsteps()
   {
      return footsteps;
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      jsonNode.put("description", description);
      jsonNode.put("swingDuration", swingDuration);
      jsonNode.put("transferDuration", transferDuration);
      jsonNode.put("parentFrame", getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (FootstepActionData footstep : footsteps)
      {
         ObjectNode footstepNode = foostepsArrayNode.addObject();
         footstep.saveToFile(footstepNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      description = jsonNode.get("description").textValue();
      swingDuration = jsonNode.get("swingDuration").asDouble();
      transferDuration = jsonNode.get("transferDuration").asDouble();
      getConditionalReferenceFrame().setParentFrameName(jsonNode.get("parentFrame").textValue());
      footsteps.clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.add().loadFromFile(footstepNode));
   }

   public void toMessage(FootstepPlanActionDescriptionMessage message)
   {
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getConditionalReferenceFrame().getConditionallyValidParentFrameName());
      message.getFootsteps().clear();
      for (FootstepActionData footstep : footsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }
   }

   public void toMessage(FootstepPlanActionDescriptionMessage message)
   {
      message.getParentFrame().resetQuick();
      message.getParentFrame().add(getParentFrame().getName());
      MessageTools.toMessage(planFrame.getTransformToParent(), message.getTransformToParent());
      message.setSwingDuration(swingDuration);
      message.setTransferDuration(transferDuration);

      message.getFootsteps().clear();
      for (FootstepActionData footstep : footsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }
   }

   public void fromMessage(FootstepPlanActionDescriptionMessage message)
   {
      swingDuration = message.getSwingDuration();
      transferDuration = message.getTransferDuration();
      getConditionalReferenceFrame().setParentFrameName(message.getParentFrame().getString(0));
      footsteps.clear();
      for (FootstepActionMessage footstep : message.getFootsteps())
      {
         footsteps.add().fromMessage(footstep);
      }
   }
}
