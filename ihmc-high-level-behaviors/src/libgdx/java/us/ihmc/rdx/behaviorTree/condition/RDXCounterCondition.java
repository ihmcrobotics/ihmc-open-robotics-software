package us.ihmc.rdx.behaviorTree.condition;

import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTBidirectionalLong;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImLongWrapper;

public class RDXCounterCondition
{
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   private final CRDTBidirectionalLong count;
   private final CRDTBidirectionalLong countTo;


   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImLongWrapper countWidget;
   private final ImLongWrapper countToWidget;

   public RDXCounterCondition(ConditionNodeState state)
   {
      this.state = state;

      definition = state.getDefinition();

      count = state.getCounter().getCount();
      countTo = definition.getCounter().getCountTo();

      countWidget = new ImLongWrapper(count::getValue,
                                      count::setValue,
                                      imLong -> ImGuiTools.volatileInputLong(labels.get("Count"), imLong));
      countToWidget = new ImLongWrapper(countTo::getValue,
                                        countTo::setValue,
                                        imLong -> ImGuiTools.volatileInputLong(labels.get("Count to"), imLong));
   }

   public void renderImGuiWidgetsInternal()
   {
      countWidget.renderImGuiWidget();
      countToWidget.renderImGuiWidget();
   }
}
