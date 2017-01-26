package us.ihmc.simulationconstructionset.gui;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public interface BookmarkedVariableRemovedListener
{
   public abstract void bookmarkRemoved(YoVariable<?> variable);
}
