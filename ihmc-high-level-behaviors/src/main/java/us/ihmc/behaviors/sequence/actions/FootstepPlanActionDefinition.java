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
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootstepPlanActionDefinition extends ActionNodeDefinition
{
   private final FootstepPlanActionDefinitionBasics footstepPlanDefinitionBasics;
   private final CRDTUnidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> footsteps;

   public FootstepPlanActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      footstepPlanDefinitionBasics = new FootstepPlanActionDefinitionBasics(crdtInfo);
      footsteps = new CRDTUnidirectionalRecyclingArrayList<>(ROS2ActorDesignation.OPERATOR,
                                                             crdtInfo,
                                                             () -> new RecyclingArrayList<>(() -> new FootstepPlanActionFootstepDefinition(crdtInfo)));
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      footstepPlanDefinitionBasics.saveToFile(jsonNode);

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

      footstepPlanDefinitionBasics.loadFromFile(jsonNode);

      footsteps.getValue().clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.getValue().add().loadFromFile(footstepNode));
   }

   public void toMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      footstepPlanDefinitionBasics.toMessage(message.getDefinitionBasics());

      message.getFootsteps().clear();
      for (int i = 0; i < footsteps.getSize(); i++)
      {
         footsteps.getValueReadOnly(i).toMessage(message.getFootsteps().add());
      }
   }

   public void fromMessage(FootstepPlanActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      footstepPlanDefinitionBasics.fromMessage(message.getDefinitionBasics());

      footsteps.fromMessage(writableList ->
      {
         writableList.clear();
         for (FootstepPlanActionFootstepDefinitionMessage footstepMessage : message.getFootsteps())
         {
            writableList.add().fromMessage(footstepMessage);
         }
      });
   }

   public FootstepPlanActionDefinitionBasics getBasics()
   {
      return footstepPlanDefinitionBasics;
   }

   public CRDTUnidirectionalRecyclingArrayList<FootstepPlanActionFootstepDefinition> getFootsteps()
   {
      return footsteps;
   }
}
