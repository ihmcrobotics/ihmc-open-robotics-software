package us.ihmc.robotics.dataStructures.listener;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public interface YoVariableRegistryChangedListener
{
   public void yoVariableWasRegistered(YoVariableRegistry registry, YoVariable<?> registeredYoVariable);
   public void yoVariableRegistryWasAdded(YoVariableRegistry addedYoVariableRegistry);
   public void yoVariableRegistryWasCleared(YoVariableRegistry clearedYoVariableRegistry);
}
