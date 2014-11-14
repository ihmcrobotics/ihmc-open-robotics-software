package us.ihmc.simulationconstructionset.robotcommprotocol;

import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariableList;


public interface CreatedNewVariablesListener
{
   public abstract void createdNewVariables(YoVariableList newVariables);
   public abstract void createdNewVariable(YoVariable variable);
}
