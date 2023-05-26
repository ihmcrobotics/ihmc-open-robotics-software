package us.ihmc.behaviors;

import java.util.Arrays;
import java.util.List;

public class BehaviorDefinition
{
   private final String name;
   private final BehaviorSupplier behaviorSupplier;
   private final List<BehaviorDefinition> subBehaviors;

   public BehaviorDefinition(String name, BehaviorSupplier behaviorSupplier, BehaviorDefinition... subBehaviors)
   {
      this.name = name;
      this.behaviorSupplier = behaviorSupplier;
      this.subBehaviors = Arrays.asList(subBehaviors);
   }

   public String getName()
   {
      return name;
   }

   public BehaviorSupplier getBehaviorSupplier()
   {
      return behaviorSupplier;
   }

   public List<BehaviorDefinition> getSubBehaviors()
   {
      return subBehaviors;
   }
}
