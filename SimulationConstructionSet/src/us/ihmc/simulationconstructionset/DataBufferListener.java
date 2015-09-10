package us.ihmc.simulationconstructionset;

import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public interface DataBufferListener
{
   public DoubleYoVariable[] getVariablesOfInterest(YoVariableHolder yoVariableHolder);

   public void dataBufferUpdate(double[] values);
}
