package us.ihmc.robotics.numericalMethods;

import static org.junit.Assert.fail;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

public class QuarticEquationSolverTest
{
   private static final boolean DEBUG = false;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testUpToQuarticEquationSolver()
   {
      QuarticEquationSolver solver = new QuarticEquationSolver();
      printIfDebug("Quatric Equation Solver.  Tests...");

      printIfDebug("Test Quadratic Equation Solver First.  Solutions to 3.0x^2 + 2.1x + 9.5 are:");
      double[] solutionRealParts = new double[4], solutionImaginaryParts = new double[4];
      solver.solveQuadraticEquation(3.0, 2.1, 9.5, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-0.35, 1.744754042, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-0.35, -1.744754042, solutionRealParts, solutionImaginaryParts);

      printIfDebug(solutionRealParts[0] + " + " + solutionImaginaryParts[0] + "i, " + solutionRealParts[1] + " + " + solutionImaginaryParts[1] + "i, ");
      printIfDebug("Answer should be   -0.3500000000 + 1.744754042*I,    -0.3500000000 - 1.744754042*I");
      printIfDebug("");

      printIfDebug("Test Cubic Equation Solver Next.  Solutions to x^3 - x^2 + x -1 are:");
      solver.solveCubicEquation(-1.0, 1.0, -1.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(0.0, 1.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(0.0, -1.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(1.0, 0.0, solutionRealParts, solutionImaginaryParts);
      
      printIfDebug(solutionRealParts[0] + " + " + solutionImaginaryParts[0] + "i, " + solutionRealParts[1] + " + " + solutionImaginaryParts[1] + "i, " + solutionRealParts[2] + " + " + solutionImaginaryParts[2] + "i, ");
      printIfDebug("Answer should be   1.0,    I,    -I");
      printIfDebug("");

      printIfDebug("Test another Cubic Equation Solver Next.  Solutions to x^3 + 6x^2 + 9x + 6 are:");
      solver.solveCubicEquation(6.0, 9.0, 6.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-4.195823345, 0.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-0.9020883273, -0.7850032632, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-0.9020883273, 0.7850032632, solutionRealParts, solutionImaginaryParts);

      printIfDebug(solutionRealParts[0] + " + " + solutionImaginaryParts[0] + "i, " + solutionRealParts[1] + " + " + solutionImaginaryParts[1] + "i, " + solutionRealParts[2] + " + " + solutionImaginaryParts[2] + "i, ");
      printIfDebug("Answer should be -4.195823345,   -.9020883273-.7850032632*I,   -.9020883273+.7850032632*I");
      printIfDebug("");

      printIfDebug("Test another Cubic Equation Solver Next.  Solutions to x^3 - 1 are:");
      solver.solveCubicEquation(0.0, 0.0, -1.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-0.5000000000, 0.8660254038, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-0.5000000000, -0.8660254038, solutionRealParts, solutionImaginaryParts);

      
      printIfDebug(solutionRealParts[0] + " + " + solutionImaginaryParts[0] + "i, " + solutionRealParts[1] + " + " + solutionImaginaryParts[1] + "i, " + solutionRealParts[2] + " + " + solutionImaginaryParts[2] + "i, ");
      printIfDebug("Answer should be 1.0,  -0.5000000000 + 0.8660254038*I,   -0.5000000000 - 0.8660254038*I");
      printIfDebug("");

      printIfDebug("Test another Cubic Equation Solver Next.  Solutions to x^3 + 2x^2 - 5x - 6 are:");
      solver.solveCubicEquation(2.0, -5.0, -6.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-1.0, 0.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(2.0, 0.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-3.0, 0.0, solutionRealParts, solutionImaginaryParts);

      printIfDebug(solutionRealParts[0] + " + " + solutionImaginaryParts[0] + "i, " + solutionRealParts[1] + " + " + solutionImaginaryParts[1] + "i, " + solutionRealParts[2] + " + " + solutionImaginaryParts[2] + "i, ");
      printIfDebug("Answer should be: -1, 2, -3");
      printIfDebug("");

   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testQuarticEquationSolver()
   {
      QuarticEquationSolver solver = new QuarticEquationSolver();
      double[] solutionRealParts = new double[4], solutionImaginaryParts = new double[4];

      printIfDebug("Test Quartic Equation Solver.  Solutions to x^4 + 6x^3 - 5x^2 - 10x -3 are:");
      solver.solveQuarticEquation(6.0, -5.0, -10.0, -3.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(1.618033989, 0.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-0.6180339888, 0.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-0.4586187348, 0.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-6.541381265, 0.0, solutionRealParts, solutionImaginaryParts);

      printIfDebug(solutionRealParts[0] + " + " + solutionImaginaryParts[0] + "i, " + solutionRealParts[1] + " + " + solutionImaginaryParts[1] + "i, " + solutionRealParts[2] + " + " + solutionImaginaryParts[2] + "i, "
                         + solutionRealParts[3] + " + " + solutionImaginaryParts[3] + "i, ");
      printIfDebug("Answer should be:  1.618033989,   -0.6180339888,   -0.4586187348,   -6.541381265");
      printIfDebug("");
      
      printIfDebug("Test Quartic Equation Solver.  Given the real solutions a, b, c, d construct the equation:");
      
      double a = 1.77;
      double b = 0.34;
      double c = 0.99;
      double d = 12.3;
      
      double x3 = a+b+c+d;
      double x2 = a*b + a*c + a*d + b*c + b*d + c*d;
      double x1 = a*b*c + a*b*d + a*c*d + b*d*c;
      double x0 = a*b*c*d;
      solver.solveQuarticEquation(x3, x2, x1, x0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-a, 0.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-b, 0.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-c, 0.0, solutionRealParts, solutionImaginaryParts);
      assertSolutionContains(-d, 0.0, solutionRealParts, solutionImaginaryParts);


   }
   
   private void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
   }
   
   private void assertSolutionContains(double realPart, double imaginaryPart, double[] realSolutions, double[] imaginarySolutions)
   {      
      for (int i=0; i<realSolutions.length; i++)
      {
         if ((Math.abs(realSolutions[i] - realPart) < 1e-7) && (Math.abs(imaginarySolutions[i] - imaginaryPart) < 1e-7)) return;
      }

      String errorMessage = "Cannot find " + realPart + " + " + imaginaryPart + " I. Solutions are: ";
      for (int i=0; i<realSolutions.length; i++)
      {
         errorMessage = errorMessage + "\n " + realSolutions[i] + " + " + imaginarySolutions[i] + " I";
      }
      fail(errorMessage);
   }

}
