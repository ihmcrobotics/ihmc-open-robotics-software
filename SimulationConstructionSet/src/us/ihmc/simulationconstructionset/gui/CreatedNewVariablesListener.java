package us.ihmc.simulationconstructionset.gui;

import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;


public interface CreatedNewVariablesListener
{
   public abstract void createdNewVariables(YoVariableList newVariables);
   public abstract void createdNewVariable(YoVariable<?> variable);
}
