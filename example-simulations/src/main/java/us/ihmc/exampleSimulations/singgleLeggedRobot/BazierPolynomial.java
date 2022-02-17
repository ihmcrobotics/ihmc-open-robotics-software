package us.ihmc.exampleSimulations.singgleLeggedRobot;

public class BazierPolynomial
{
   MathForML mathTool;

   public BazierPolynomial()
   {
      mathTool = new MathForML();
   }

   public double BazierPolynomialBasis(double x, int i, int n)
   {
      return mathTool.factorial(n) / mathTool.factorial(i) / mathTool.factorial(n - i) * Math.pow(x, i) * Math.pow(1 - x, n - i);
   }

   public double BazierPolynomialFunction(double x, MatrixForML bazierCoefficient)
   {
      int n = bazierCoefficient.getCol() - 1;
      double temp = 0;
      for (int i = 0; i < n; i++)
      {
         temp += bazierCoefficient.getDoubleValue(0, i) * BazierPolynomialBasis(x, i, n);
      }

      return temp;
   }

   public double dBazierPolynomial(double x, double stanceTime, MatrixForML bazierCoefficient)
   {
      int n = bazierCoefficient.getCol() - 1;
      double temp = 0;
      for (int i = 0; i < n; i++)
      {
         temp +=  n / stanceTime * BazierPolynomialBasis(x, i, n - 1) * (bazierCoefficient.getDoubleValue(0,i + 1) - bazierCoefficient.getDoubleValue(0,i));
      }

      return temp;
   }
}
