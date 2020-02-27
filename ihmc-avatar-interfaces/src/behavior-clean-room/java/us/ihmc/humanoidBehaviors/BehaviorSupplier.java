package us.ihmc.humanoidBehaviors;

import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;

public interface BehaviorSupplier
{
   BehaviorInterface build(BehaviorHelper helper);
}
