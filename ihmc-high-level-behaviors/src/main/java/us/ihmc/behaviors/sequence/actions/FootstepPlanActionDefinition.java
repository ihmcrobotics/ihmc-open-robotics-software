package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage;
import behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootstepPlanActionDefinition extends ActionNodeDefinition
{
   private final CRDTUnidirectionalDouble swingDuration;
   private final CRDTUnidirectionalDouble transferDuration;
   private final CRDTUnidirectionalString parentFrameName;
   private final CRDTUnidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> footsteps;

   public FootstepPlanActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      swingDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.2);
      transferDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.8);
      parentFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      footsteps = new CRDTUnidirectionalRecyclingArrayList<>(ROS2ActorDesignation.OPERATOR,
                                                             crdtInfo,
                                                             () -> new RecyclingArrayList<>(() -> new FootstepPlanActionFootstepDefinition(crdtInfo)));
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("swingDuration", swingDuration.getValue());
      jsonNode.put("transferDuration", transferDuration.getValue());
      jsonNode.put("parentFrame", parentFrameName.getValue());

      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (int i = 0; i < footsteps.getSize(); i++)
      {
         ObjectNode footstepNode = foostepsArrayNode.addObject();
         footsteps.getValueReadOnly(i).saveToFile(footstepNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      swingDuration.setValue(jsonNode.get("swingDuration").asDouble());
      transferDuration.setValue(jsonNode.get("transferDuration").asDouble());
      parentFrameName.setValue(jsonNode.get("parentFrame").textValue());

      footsteps.getValue().clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.getValue().add().loadFromFile(footstepNode));
   }

   public void toMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setSwingDuration(swingDuration.toMessage());
      message.setTransferDuration(transferDuration.toMessage());
      message.setParentFrameName(parentFrameName.toMessage());

      message.getFootsteps().clear();
      for (int i = 0; i < footsteps.getSize(); i++)
      {
         footsteps.getValueReadOnly(i).toMessage(message.getFootsteps().add());
      }
   }

   public void fromMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      swingDuration.fromMessage(message.getSwingDuration());
      transferDuration.fromMessage(message.getTransferDuration());
      parentFrameName.fromMessage(message.getParentFrameNameAsString());

      footsteps.fromMessage(writableList ->
      {
         if (message.getFootsteps().size() != writableList.size())
            LogTools.info("%d -> %d".formatted(writableList.size(), message.getFootsteps().size()));
         writableList.clear();
         for (FootstepPlanActionFootstepDefinitionMessage footstepMessage : message.getFootsteps())
         {
            writableList.add().fromMessage(footstepMessage);
         }
      });
   }

   public double getSwingDuration()
   {
      return swingDuration.getValue();
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration.setValue(swingDuration);
   }

   public double getTransferDuration()
   {
      return transferDuration.getValue();
   }

   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration.setValue(transferDuration);
   }

   public String getParentFrameName()
   {
      return parentFrameName.getValue();
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName.setValue(parentFrameName);
   }

   public CRDTUnidirectionalString getCRDTParentFrameName()
   {
      return parentFrameName;
   }

   public CRDTUnidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> getFootsteps()
   {
      return footsteps;
   }
}
