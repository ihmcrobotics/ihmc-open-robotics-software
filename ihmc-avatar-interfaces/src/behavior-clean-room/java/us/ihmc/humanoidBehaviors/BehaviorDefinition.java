package us.ihmc.humanoidBehaviors;

import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;

public class BehaviorDefinition
{
   private final String name;
   private final BehaviorSupplier behaviorSupplier;
   private final MessagerAPI behaviorAPI;

   public BehaviorDefinition(String name, BehaviorSupplier behaviorSupplier, MessagerAPI behaviorAPI)
   {
      this.name = name;
      this.behaviorSupplier = behaviorSupplier;
      this.behaviorAPI = behaviorAPI;
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
}
