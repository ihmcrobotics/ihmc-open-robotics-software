package us.ihmc.behaviors;

import us.ihmc.behaviors.tools.BehaviorHelper;

/**
 * A behavior's constructor should implement this interface enabling
 * to pass a behavior constructor as i.e. SomeBehavior::new
 */
public interface BehaviorSupplier
{
   BehaviorInterface build(BehaviorHelper helper);
}
