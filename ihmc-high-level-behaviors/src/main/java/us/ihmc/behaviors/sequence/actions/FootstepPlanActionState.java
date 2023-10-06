package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage;
import behavior_msgs.msg.dds.FootstepPlanActionStateMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.JSONTools;

public class FootstepPlanActionState extends BehaviorActionState
{
   private final FootstepPlanActionDefinition definition = new FootstepPlanActionDefinition();
   private final RecyclingArrayList<FootstepPlanActionFootstepState> footsteps;

   public FootstepPlanActionState(ReferenceFrameLibrary referenceFrameLibrary)
   {
      footsteps = new RecyclingArrayList<>(() -> new FootstepPlanActionFootstepState(referenceFrameLibrary, this));
   }

   @Override
   public void update()
   {
      for (int i = 0; i < footsteps.size(); i++)
      {
         footsteps.get(i).setIndex(i);
         footsteps.get(i).update();
      }
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      ArrayNode foostepsArrayNode = jsonNode.putArray("footsteps");
      for (FootstepPlanActionFootstepState footstep : footsteps)
      {
         ObjectNode footstepNode = foostepsArrayNode.addObject();
         footstep.getDefinition().saveToFile(footstepNode);
      }
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      footsteps.clear();
      JSONTools.forEachArrayElement(jsonNode, "footsteps", footstepNode -> footsteps.add().getDefinition().loadFromFile(footstepNode));
   }

   public void toMessage(FootstepPlanActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());

      message.getFootsteps().clear();
      for (FootstepPlanActionFootstepState footstep : footsteps)
      {
         footstep.toMessage(message.getFootsteps().add());
      }
   }

   public void fromMessage(FootstepPlanActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());

      footsteps.clear();
      for (FootstepPlanActionFootstepStateMessage footstep : message.getFootsteps())
      {
         footsteps.add().fromMessage(footstep);
      }
   }

   public RecyclingArrayList<FootstepPlanActionFootstepState> getFootsteps()
   {
      return footsteps;
   }

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }
}
