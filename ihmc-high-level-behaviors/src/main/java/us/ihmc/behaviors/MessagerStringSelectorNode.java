package us.ihmc.behaviors;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNode;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static us.ihmc.behaviors.BehaviorModule.API.BehaviorSelection;

/**
 * Not finished or tested. Mostly to hold old code.
 */
public class MessagerStringSelectorNode extends BehaviorTreeControlFlowNode implements BehaviorInterface
{
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private final Map<String, Pair<BehaviorDefinition, BehaviorInterface>> constructedBehaviors = new HashMap<>();
   private final Map<String, Boolean> enabledBehaviors = new HashMap<>();
   private final BehaviorHelper helper;
   private final StatusLogger statusLogger;

   public MessagerStringSelectorNode(BehaviorRegistry behaviorRegistry, BehaviorHelper behaviorHelper)
   {
      helper = behaviorHelper;
      statusLogger = helper.getOrCreateStatusLogger();

      for (BehaviorDefinition behaviorDefinition : behaviorRegistry.getDefinitionEntries())
      {
         BehaviorInterface constructedBehavior = behaviorDefinition.getBehaviorSupplier().build(helper);
         constructedBehaviors.put(behaviorDefinition.getName(), Pair.of(behaviorDefinition, constructedBehavior));
         YoRegistry yoRegistry = constructedBehavior.getYoRegistry();
         if (yoRegistry != null)
         {
            this.yoRegistry.addChild(yoRegistry);
         }
      }

      helper.subscribeViaCallback(BehaviorSelection, this::stringBasedSelection);
   }

   private void stringBasedSelection(String selection)
   {
      ArrayList<String> selectedBehaviors = new ArrayList<>();
      selectedBehaviors.add(selection);
      if (constructedBehaviors.containsKey(selection)) // i.e. Might be "None"
      {
         for (BehaviorDefinition subBehavior : constructedBehaviors.get(selection).getLeft().getSubBehaviors())
         {
            selectedBehaviors.add(subBehavior.getName());
         }
      }

      boolean selectedOne = false;
      for (Map.Entry<String, Pair<BehaviorDefinition, BehaviorInterface>> behavior : constructedBehaviors.entrySet())
      {
         String behaviorName = behavior.getKey();
         boolean selected = selectedBehaviors.contains(behaviorName);
         if (selected)
         {
            selectedOne = true;
         }
         if (enabledBehaviors.computeIfAbsent(behaviorName, key -> false) != selected)
         {
            enabledBehaviors.put(behaviorName, selected);
            statusLogger.info("{} {} behavior.", selected ? "Enabling" : "Disabling", behaviorName);
            behavior.getValue().getRight().setEnabled(selected);
         }
      }
      if (!selectedOne)
      {
         statusLogger.info("All behaviors disabled.");
      }
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      return BehaviorTreeNodeStatus.SUCCESS;
   }

   @Override
   public void setEnabled(boolean enabled)
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return yoRegistry;
   }

   @Override
   public void destroy()
   {
      for (Pair<BehaviorDefinition, BehaviorInterface> behavior : constructedBehaviors.values())
      {
         behavior.getRight().destroy();
      }
   }
}
