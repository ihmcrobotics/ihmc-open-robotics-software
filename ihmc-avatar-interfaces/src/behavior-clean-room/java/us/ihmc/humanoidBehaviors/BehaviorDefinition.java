package us.ihmc.humanoidBehaviors;

import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;

import java.util.Arrays;
import java.util.List;

public class BehaviorDefinition
{
   private final String name;
   private final BehaviorSupplier behaviorSupplier;
   private final MessagerAPI behaviorAPI;
   private final List<BehaviorDefinition> subBehaviors;

   public BehaviorDefinition(String name, BehaviorSupplier behaviorSupplier, MessagerAPI behaviorAPI, BehaviorDefinition... subBehaviors)
   {
      this.name = name;
      this.behaviorSupplier = behaviorSupplier;
      this.behaviorAPI = behaviorAPI;
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

   public MessagerAPI getBehaviorAPI()
   {
      return behaviorAPI;
   }

   public List<BehaviorDefinition> getSubBehaviors()
   {
      return subBehaviors;
   }
}
