package us.ihmc.humanoidBehaviors;

import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;

/**
 * A behavior's constructor should implement this interface enabling
 * to pass a behavior constructor as i.e. SomeBehavior::new
 */
public interface BehaviorSupplier
{
   BehaviorInterface build(BehaviorHelper helper);
}
