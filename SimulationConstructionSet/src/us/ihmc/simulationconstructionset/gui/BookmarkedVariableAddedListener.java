package us.ihmc.simulationconstructionset.gui;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public interface BookmarkedVariableAddedListener
{
   public abstract void bookmarkAdded(YoVariable<?> variable);
}
