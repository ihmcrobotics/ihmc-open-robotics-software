package us.ihmc.simulationconstructionset;

import us.ihmc.yoUtilities.dataStructure.YoVariableHolder;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public interface DataBufferListener
{
   public DoubleYoVariable[] getVariablesOfInterest(YoVariableHolder yoVariableHolder);

   public void dataBufferUpdate(double[] values);
}
