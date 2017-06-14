package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface CoactiveElement
{
   // User Interface Side:
   public abstract void initializeUserInterfaceSide();
   public abstract void updateUserInterfaceSide();
   public abstract YoVariableRegistry getUserInterfaceWritableYoVariableRegistry();

   // Robot Side:
   public abstract void initializeMachineSide();
   public abstract void updateMachineSide();
   public abstract YoVariableRegistry getMachineWritableYoVariableRegistry();
}
