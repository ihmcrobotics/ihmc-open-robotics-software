package us.ihmc.simulationconstructionset.gui;

import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.variable.YoVariableList;


public interface CreatedNewVariablesListener
{
   public abstract void createdNewVariables(YoVariableList newVariables);
   public abstract void createdNewVariable(YoVariable<?> variable);
}
