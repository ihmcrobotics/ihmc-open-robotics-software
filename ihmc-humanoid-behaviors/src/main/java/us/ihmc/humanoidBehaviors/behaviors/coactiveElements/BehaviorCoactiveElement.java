package us.ihmc.humanoidBehaviors.behaviors.coactiveElements;

import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BehaviorCoactiveElement implements CoactiveElement
{
   protected final YoVariableRegistry userInterfaceWritableRegistry = new YoVariableRegistry("UserInterfaceSide");
   protected final YoVariableRegistry machineWritableRegistry = new YoVariableRegistry("MachineSide");

   @Override
   public void initializeUserInterfaceSide()
   {

   }

   @Override
   public void updateUserInterfaceSide()
   {

   }

   @Override
   public YoVariableRegistry getUserInterfaceWritableYoVariableRegistry()
   {
      return userInterfaceWritableRegistry;
   }

   @Override
   public void initializeMachineSide()
   {

   }

   @Override
   public void updateMachineSide()
   {

   }

   @Override
   public YoVariableRegistry getMachineWritableYoVariableRegistry()
   {
      return machineWritableRegistry;
   }
}
