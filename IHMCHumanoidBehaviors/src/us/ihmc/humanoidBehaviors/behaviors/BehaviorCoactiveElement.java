package us.ihmc.humanoidBehaviors.behaviors;

import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public abstract class BehaviorCoactiveElement implements CoactiveElement
{
   protected final YoVariableRegistry userInterfaceWritableRegistry = new YoVariableRegistry("UserInterfaceSide");
   protected final YoVariableRegistry machineWritableRegistry = new YoVariableRegistry("MachineSide");

   @Override
   public YoVariableRegistry getUserInterfaceWritableYoVariableRegistry()
   {
      return userInterfaceWritableRegistry;
   }

   @Override
   public YoVariableRegistry getMachineWritableYoVariableRegistry()
   {
      return machineWritableRegistry;
   }
}
