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
      System.err.println("BehaviorCoactiveElement - initializeUserInterfaceSide not implemented");
   }

   @Override
   public void updateUserInterfaceSide()
   {
      System.err.println("BehaviorCoactiveElement - updateUserInterfaceSide not implemented");
   }

   @Override
   public YoVariableRegistry getUserInterfaceWritableYoVariableRegistry()
   {
      return userInterfaceWritableRegistry;
   }

   @Override
   public void initializeMachineSide()
   {
      System.err.println("BehaviorCoactiveElement - initializeMachineSide not implemented");

   }

   @Override
   public void updateMachineSide()
   {
      System.err.println("BehaviorCoactiveElement - updateMachineSide not implemented");

   }

   @Override
   public YoVariableRegistry getMachineWritableYoVariableRegistry()
   {
      return machineWritableRegistry;
   }
}
