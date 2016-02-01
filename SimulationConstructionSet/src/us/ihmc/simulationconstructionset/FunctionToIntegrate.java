
package us.ihmc.simulationconstructionset;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public interface FunctionToIntegrate
{
   public double[] computeDerivativeVector();

   public int getVectorSize();

   public DoubleYoVariable[] getOutputVariables();
}
