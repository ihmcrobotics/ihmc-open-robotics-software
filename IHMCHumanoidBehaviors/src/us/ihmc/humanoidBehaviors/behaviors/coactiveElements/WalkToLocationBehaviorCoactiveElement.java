package us.ihmc.humanoidBehaviors.behaviors.coactiveElements;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public abstract class WalkToLocationBehaviorCoactiveElement extends BehaviorCoactiveElement
{


   //UI SIDE YOVARS
   public final DoubleYoVariable x = new DoubleYoVariable("x", userInterfaceWritableRegistry);
   public final DoubleYoVariable y = new DoubleYoVariable("y", userInterfaceWritableRegistry);
   public final DoubleYoVariable z = new DoubleYoVariable("z", userInterfaceWritableRegistry);

   public final BooleanYoVariable locationSet = new BooleanYoVariable("locationSet", userInterfaceWritableRegistry);

   //BEHAVIOR SIDE YOVARS

   public final BooleanYoVariable walking = new BooleanYoVariable("walking", machineWritableRegistry);
   public final BooleanYoVariable locationReached = new BooleanYoVariable("locationReached", machineWritableRegistry);

}
