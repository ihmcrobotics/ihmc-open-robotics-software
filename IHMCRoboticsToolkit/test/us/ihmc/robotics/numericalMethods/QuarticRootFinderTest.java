package us.ihmc.robotics.numericalMethods;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.junit.Test;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.MutationTestingTools;

public class QuarticRootFinderTest
{
   private static final boolean DEBUG = false;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testQuarticRootFinder()
   {
      double[] realSolutions = new double[4];
      int numberOfRealSolutions = 0;

      QuarticRootFinder solver = new QuarticRootFinder();
      printIfDebug("Quatric Root Finder.  Tests...");

      printIfDebug("Test Quadratic Equation Solver First.  Solutions to 3.0x^2 + 2.1x + 9.5 are:");
      numberOfRealSolutions = solver.SolveQuadric(new double[] { 9.5, 2.1, 3.0 }, realSolutions);
      assertEquals(0, numberOfRealSolutions);
      printSolution(numberOfRealSolutions, realSolutions);
      printIfDebug("Answer should be   -0.3500000000 + 1.744754042*I,    -0.3500000000 - 1.744754042*I");
      printIfDebug("");

      printIfDebug("Test Cubic Equation Solver Next.  Solutions to x^3 - x^2 + x -1 are:");
      numberOfRealSolutions = solver.SolveCubic(new double[] { -1.0, 1.0, -1.0, 1.0 }, realSolutions);
      assertEquals(1, numberOfRealSolutions);
      assertSolutionContains(1.0, realSolutions);
      printSolution(numberOfRealSolutions, realSolutions);
      printIfDebug("Answer should be   1.0,    I,    -I");
      printIfDebug("");

      printIfDebug("Test another Cubic Equation Solver Next.  Solutions to x^3 + 6x^2 + 9x + 6 are:");
      numberOfRealSolutions = solver.SolveCubic(new double[] { 6.0, 9.0, 6.0, 1.0 }, realSolutions);
      assertEquals(1, numberOfRealSolutions);
      assertSolutionContains(-4.195823345, realSolutions);
      printSolution(numberOfRealSolutions, realSolutions);
      printIfDebug("Answer should be -4.195823345,   -.9020883273-.7850032632*I,   -.9020883273+.7850032632*I");
      printIfDebug("");

      printIfDebug("Test another Cubic Equation Solver Next.  Solutions to x^3 - 1 are:");
      numberOfRealSolutions = solver.SolveCubic(new double[] { -1.0, 0.0, 0.0, 1.0 }, realSolutions);
      assertEquals(1, numberOfRealSolutions);
      assertSolutionContains(1.0, realSolutions);
      printSolution(numberOfRealSolutions, realSolutions);
      printIfDebug("Answer should be 1.0,  -0.5000000000 + 0.8660254038*I,   -0.5000000000 - 0.8660254038*I");
      printIfDebug("");

      printIfDebug("Test another Cubic Equation Solver Next.  Solutions to x^3 + 2x^2 - 5x - 6 are:");
      numberOfRealSolutions = solver.SolveCubic(new double[] { -6.0, -5.0, 2.0, 1.0 }, realSolutions);
      assertEquals(3, numberOfRealSolutions);
      assertSolutionContains(-1.0, realSolutions);
      assertSolutionContains(2.0, realSolutions);
      assertSolutionContains(-3.0, realSolutions);
      printSolution(numberOfRealSolutions, realSolutions);
      printIfDebug("Answer should be: -1, 2, -3");
      printIfDebug("");

      printIfDebug("Test Quartic Equation Solver .  Solutions to x^4 + 6x^3 - 5x^2 - 10x -3 are:");
      numberOfRealSolutions = solver.SolveQuartic(new double[] { -3.0, -10.0, -5.0, 6.0, 1.0 }, realSolutions);
      assertEquals(4, numberOfRealSolutions);
      assertSolutionContains(1.618033989, realSolutions);
      assertSolutionContains(-0.6180339888, realSolutions);
      assertSolutionContains(-0.4586187348, realSolutions);
      assertSolutionContains(-6.541381265, realSolutions);
      printIfDebug("There are " + numberOfRealSolutions + " real valued solutions.  They are : ");
      printSolution(numberOfRealSolutions, realSolutions);
      printIfDebug("Answer should be:  1.618033989,   -0.6180339888,   -0.4586187348,   -6.541381265");
      printIfDebug("");
   }

   private static void printSolution(int num, double[] real_solutions)
   {
      for (int i = 0; i < num; i++)
      {
         if (DEBUG)
            System.out.print(real_solutions[i] + "   ");
      }

      if (DEBUG)
         System.out.println();
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

   private void assertSolutionContains(double realPart, double[] realSolutions)
   {
      for (int i = 0; i < realSolutions.length; i++)
      {
         if (Math.abs(realSolutions[i] - realPart) < 1e-7)
            return;
      }

      String errorMessage = "Cannot find " + realPart + " Solutions are: ";
      for (int i = 0; i < realSolutions.length; i++)
      {
         errorMessage = errorMessage + "\n " + realSolutions[i];
      }
      fail(errorMessage);
   }

   public static void main(String[] args)
   {
      String targetTests = QuarticRootFinderTest.class.getName();
      String targetClass = QuarticRootFinder.class.getName();
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClass);
   }
}
