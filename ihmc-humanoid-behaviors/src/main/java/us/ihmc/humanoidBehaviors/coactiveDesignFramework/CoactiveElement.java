package us.ihmc.humanoidBehaviors.coactiveDesignFramework;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface CoactiveElement
{
   void initializeUserInterfaceSide();

   void updateUserInterfaceSide();

   YoVariableRegistry getUserInterfaceWritableYoVariableRegistry();

   void initializeMachineSide();

   void updateMachineSide();

   YoVariableRegistry getMachineWritableYoVariableRegistry();
}
