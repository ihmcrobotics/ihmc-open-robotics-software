package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public interface RegistrySettingsChangedListener
{
   public abstract void registrySettingsChanged(ArrayList<YoVariableRegistry> changedRegistries);

   public abstract void registrySettingsChanged();
}
