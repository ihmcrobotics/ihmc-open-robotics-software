
package us.ihmc.simulationconstructionset;

import us.ihmc.yoVariables.variable.YoDouble;


public interface FunctionToIntegrate
{
   public double[] computeDerivativeVector();

   public int getVectorSize();

   public YoDouble[] getOutputVariables();
}
