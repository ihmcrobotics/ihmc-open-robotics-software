package us.ihmc.simulationconstructionset;

import us.ihmc.yoVariables.variable.YoDouble;


public class FunctionIntegrator
{
   private FunctionToIntegrate function;
   private double[] q_n;
   private double[][] k;
   private int vectorSize;

   private double[] tempDerivative;

   private YoDouble[] outputs;


   public FunctionIntegrator(FunctionToIntegrate function)
   {
      this.function = function;

      vectorSize = function.getVectorSize();

      q_n = new double[vectorSize];
      k = new double[4][vectorSize];

      outputs = function.getOutputVariables();
   }

   public void saveTempState()
   {
      for (int i = 0; i < vectorSize; i++)
      {
         q_n[i] = outputs[i].getDoubleValue();
      }
   }


   public void doDynamics(int passNumber)
   {
      tempDerivative = function.computeDerivativeVector();

      for (int i = 0; i < vectorSize; i++)
      {
         k[passNumber][i] = tempDerivative[i];
      }
   }

   public void restoreTempState()
   {
      for (int i = 0; i < vectorSize; i++)
      {
         outputs[i].set(q_n[i]);
      }
   }

   public void eulerIntegrate(double stepSize)
   {
      for (int i = 0; i < vectorSize; i++)
      {
         outputs[i].set(outputs[i].getDoubleValue() + stepSize * tempDerivative[i]);
      }
   }

   public void rungeKuttaSum(double stepSize)
   {
      for (int i = 0; i < vectorSize; i++)
      {
         outputs[i].set(q_n[i] + stepSize * (k[0][i] / 6.0 + k[1][i] / 3.0 + k[2][i] / 3.0 + k[3][i] / 6.0));
      }
   }


}
