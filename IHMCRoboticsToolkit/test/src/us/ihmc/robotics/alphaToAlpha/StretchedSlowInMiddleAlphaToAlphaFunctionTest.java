package us.ihmc.robotics.alphaToAlpha;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class StretchedSlowInMiddleAlphaToAlphaFunctionTest
{
   public StretchedSlowInMiddleAlphaToAlphaFunctionTest()
   {
   }

   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDerivative()
   {
      double derivatesAtStartAndEnd = 10.0;
      StretchedSlowInMiddleAlphaToAlphaFunction stretchedSlowInMiddleAlphaToAlphaFunction =
         new StretchedSlowInMiddleAlphaToAlphaFunction(derivatesAtStartAndEnd);

      double alphaStart = -0.5;
      double alphaEnd = 1.5;
      double stepSize = 1e-3;
      int numberOfSteps = (int) ((alphaEnd - alphaStart) / stepSize);

      double[] alpha = new double[numberOfSteps];
      double[][] alphaPrime = new double[1][numberOfSteps];
      double[][] derivatives = new double[2][numberOfSteps];
      double[][] secondDerivatives = new double[2][numberOfSteps];

      for (int i = 0; i < numberOfSteps; i++)
      {
         double currentAlpha = alphaStart + stepSize * i;
         alpha[i] = currentAlpha;
         alphaPrime[0][i] = stretchedSlowInMiddleAlphaToAlphaFunction.getAlphaPrime(currentAlpha);
         derivatives[0][i] = stretchedSlowInMiddleAlphaToAlphaFunction.getDerivativeAtAlpha(currentAlpha);
         secondDerivatives[0][i] = stretchedSlowInMiddleAlphaToAlphaFunction.getSecondDerivativeAtAlpha(currentAlpha);
      }

      derivatives[1] = getNumericalDerivative(alphaPrime[0], stepSize);
      secondDerivatives[1] = getNumericalDerivative(derivatives[0], stepSize);

      String nameSuffix = " for a stretchedSlowInMiddleAlphaToAlphaFunction with end derivatives " + derivatesAtStartAndEnd;

      plot(alpha, alphaPrime, "AlphaPrime" + nameSuffix);
      plot(alpha, derivatives, "Analytical and numerical derivative" + nameSuffix);
      plot(alpha, secondDerivatives, "Analytical and numerical second derivative" + nameSuffix);

//    sleepForever();
   }

   private double[] getNumericalDerivative(double[] array, double stepSize)
   {
      int n = array.length;
      double[] ret = new double[n];
      for (int i = 0; i < (n - 1); i++)
      {
         ret[i] = (array[i + 1] - array[i]) / stepSize;
      }

      ret[n - 1] = Double.NaN;

      return ret;
   }

   private void plot(double[] abscissa, double[][] ordinate, String name)
   {
      double numberOfTrajectories = ordinate.length;

      ArrayList<double[][]> listOfCurves1 = new ArrayList<double[][]>();
      for (int i = 0; i < numberOfTrajectories; i++)
      {
         listOfCurves1.add(new double[][]
         {
            abscissa, ordinate[i]
         });
      }

//    PlotGraph2d pg1 = PlotGraph2d.createPlotGraph2dMultipleCurves(listOfCurves1);
//    pg1.plot();
//    pg1.setGraphTitle(name);
   }

   @SuppressWarnings("unused")
   private void sleepForever()
   {
      while (true)
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException ex)
         {
         }
      }
   }
}
