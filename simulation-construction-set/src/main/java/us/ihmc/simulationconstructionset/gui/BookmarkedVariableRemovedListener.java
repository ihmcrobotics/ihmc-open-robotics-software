package us.ihmc.simulationconstructionset.gui;

import us.ihmc.yoVariables.variable.YoVariable;

public interface BookmarkedVariableRemovedListener
{
   public abstract void bookmarkRemoved(YoVariable<?> variable);
}
