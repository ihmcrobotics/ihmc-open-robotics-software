package us.ihmc.simulationconstructionset.gui;

import us.ihmc.yoVariables.variable.YoVariable;

public interface BookmarkedVariableAddedListener
{
   public abstract void bookmarkAdded(YoVariable<?> variable);
}
