package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class HysteresisFilteredYoVariableTest
{
   YoVariableRegistry registry = new YoVariableRegistry("TestHysteresisFilteredYoVariable");
   private DoubleYoVariable guideLineHysteresis = new DoubleYoVariable("guideLineHyst", registry);
   private HysteresisFilteredYoVariable filteredYoVariable = new HysteresisFilteredYoVariable("test", registry, guideLineHysteresis);
   private double epsilon = 1e-7;

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
   public void testNoHysteresis()
   {
      guideLineHysteresis.set(0.0);

      double stepSize = Math.PI / 100.0;
      double maximumValue = Math.PI * 2.0;
      double[] x = getAbscissa(stepSize, maximumValue);

      double[] unfilteredValues = sin(x);

      double[] filteredValues = filter(unfilteredValues, filteredYoVariable);

      int n = filteredValues.length;
      for (int i = 0; i < n; i++)
      {
         assertTrue(Math.abs(filteredValues[i] - unfilteredValues[i]) < epsilon);
      }

      //    plot(x, new double[][]{unfilteredValues, filteredValues}, "No Hysteresis");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomeHysteresis()
   {
      guideLineHysteresis.set(0.2);

      double stepSize = Math.PI / 50.0;
      double maximumValue = Math.PI * 4.0;
      double[] x = getAbscissa(stepSize, maximumValue);

      double[] unfilteredValues = sin(x);

      double[] filteredValues = filter(unfilteredValues, filteredYoVariable);

      @SuppressWarnings("unused")
      int n = filteredValues.length;

      //    plot(x, new double[][]{unfilteredValues, filteredValues}, "Hysteresis = " + guideLineHysteresis.val);
   }

   private double[] getAbscissa(double stepSize, double maximumValue)
   {
      int numberOfSteps = (int) (maximumValue / stepSize);
      double[] ret = new double[numberOfSteps];
      for (int i = 0; i < numberOfSteps; i++)
      {
         ret[i] = i * stepSize;
      }

      return ret;
   }

   private double[] sin(double[] abscissa)
   {
      int n = abscissa.length;
      double[] ret = new double[n];
      for (int i = 0; i < n; i++)
      {
         ret[i] = Math.sin(abscissa[i]);
      }

      return ret;
   }

   private double[] filter(double[] unfilteredValues, HysteresisFilteredYoVariable hysteresisFilteredYoVariable)
   {
      int n = unfilteredValues.length;
      double[] ret = new double[n];
      for (int i = 0; i < n; i++)
      {
         hysteresisFilteredYoVariable.update(unfilteredValues[i]);
         ret[i] = hysteresisFilteredYoVariable.getDoubleValue();
      }

      return ret;
   }

   // private void plot(double[] abscissa, double[][] ordinates, String name)
   // {
   //    double numberOfTrajectories = ordinates.length;
   //
   //    ArrayList<double[][]> listOfCurves1 = new ArrayList<double[][]> ();
   //    for (int i = 0; i < numberOfTrajectories; i++)
   //    {
   //       listOfCurves1.add(new double[][]
   //                         {abscissa, ordinates[i]});
   //    }
   //
   //    PlotGraph2d pg1 = PlotGraph2d.createPlotGraph2dMultipleCurves(listOfCurves1);
   //    pg1.plot();
   //    pg1.setGraphTitle(name);
   //
   //    sleepForever();
   // }
   //
   //   @SuppressWarnings("unused")
   //   private void sleepForever()
   //   {
   //      while (true)
   //      {
   //         try
   //         {
   //            Thread.sleep(1000);
   //         }
   //         catch (InterruptedException ex)
   //         {
   //         }
   //      }
   //   }
}
