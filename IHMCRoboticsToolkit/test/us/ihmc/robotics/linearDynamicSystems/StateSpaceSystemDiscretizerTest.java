package us.ihmc.robotics.linearDynamicSystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.swing.JFrame;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.RandomMatrices;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.AxisLocation;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.CombinedDomainXYPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.StandardXYItemRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.testing.JUnitTools;

public class StateSpaceSystemDiscretizerTest
{
   private static final boolean DEBUG = false;
   private static final boolean DISPLAY_GRAPHS_AND_SLEEP_FOREVER = false;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testWithSimpleSpringDamperSystem()
   {
      int numberOfStates = 2;
      int numberOfInputs = 1;
      StateSpaceSystemDiscretizer stateSpaceSystemDiscretizer = new SingleMatrixExponentialStateSpaceSystemDiscretizer(numberOfStates, numberOfInputs);

      double springK = 100.0;
      double damperB = 1.0;
      double discretizationTimeStep = 0.01;
      double totalSimulationTime = 5.0;

      double xProcessCovariance = 0.1;
      double xDotProcessCovariance = 0.012;
      double xSensorCovariance = 0.03;

      double xInitial = 1.0;
      double xDotInitial = 0.1;

      DenseMatrix64F continuousA = new DenseMatrix64F(new double[][] { { 0.0, 1.0 }, { -springK, -damperB } });
      DenseMatrix64F continuousB = new DenseMatrix64F(new double[][] { { 0.0 }, { 1.0 } });

      DenseMatrix64F continuousQ = new DenseMatrix64F(new double[][] { { xProcessCovariance, 0.0 }, { 0.0, xDotProcessCovariance } });
      DenseMatrix64F continuousR = new DenseMatrix64F(new double[][] { { xSensorCovariance } });

      printSystemMatrices("Continuous: ", continuousA, continuousB, continuousQ, continuousR, 1.0);

      // Trivial discreization: discreteA = I + A*deltaT, discreteB = B*deltaT, 
      // discreteQ = Q * deltaQ, discreteR = R
      DenseMatrix64F simpleDiscreteA = new DenseMatrix64F(continuousA);
      DenseMatrix64F simpleDiscreteB = new DenseMatrix64F(continuousB);
      DenseMatrix64F simpleDiscreteQ = new DenseMatrix64F(continuousQ);
      DenseMatrix64F simpleDiscreteR = new DenseMatrix64F(continuousR);

      // simpleA = I + A
      CommonOps.scale(discretizationTimeStep, simpleDiscreteA);
      CommonOps.add(simpleDiscreteA, CommonOps.identity(numberOfStates), simpleDiscreteA);

      // simpleB = B  * deltaT
      CommonOps.scale(discretizationTimeStep, simpleDiscreteB);

      // simpleQ = Q times deltaT
      CommonOps.scale(discretizationTimeStep, simpleDiscreteQ);

      // simpleR = R
      printSystemMatrices("Simple Discrete: ", simpleDiscreteA, simpleDiscreteB, simpleDiscreteQ, simpleDiscreteR, discretizationTimeStep);

      // Discretization using StateSpaceSystemDiscretizer:
      DenseMatrix64F discreteA = new DenseMatrix64F(continuousA);
      DenseMatrix64F discreteB = new DenseMatrix64F(continuousB);

      DenseMatrix64F discreteQ = new DenseMatrix64F(continuousQ);
      DenseMatrix64F discreteR = new DenseMatrix64F(continuousR);

      stateSpaceSystemDiscretizer.discretize(discreteA, discreteB, discreteQ, discretizationTimeStep);

      JUnitTools.assertMatrixEquals(discreteR, continuousR, 1e-7);

      printSystemMatrices("Discrete: ", discreteA, discreteB, discreteQ, discreteR, discretizationTimeStep);

      int numberOfPoints = (int) (totalSimulationTime / discretizationTimeStep);
      int numberOfIntegrationSteps = 1000;
      double eulerStepSize = discretizationTimeStep / ((double) numberOfIntegrationSteps);

      double[] time = new double[numberOfPoints];

      double[] xContinuous = new double[numberOfPoints];
      double[] xDiscreteSimple = new double[numberOfPoints];
      double[] xDiscrete = new double[numberOfPoints];

      double t = 0.0, x = xInitial, xDot = xDotInitial, u;//
      //TODO: Verify scalilng of Covariance Matrices: w, v;

      double uAmplitude = 2.5;
      double uFreq = 2.0;

      DenseMatrix64F stateDiscreteSimple = new DenseMatrix64F(new double[][] { { xInitial }, { xDotInitial } });
      DenseMatrix64F stateDiscrete = new DenseMatrix64F(new double[][] { { xInitial }, { xDotInitial } });
      DenseMatrix64F input = new DenseMatrix64F(1, 1);

      for (int pointNumber = 0; pointNumber < numberOfPoints; pointNumber++)
      {
         time[pointNumber] = t;
         xContinuous[pointNumber] = x;
         xDiscreteSimple[pointNumber] = stateDiscreteSimple.get(0, 0);
         xDiscrete[pointNumber] = stateDiscrete.get(0, 0);

         // Simulate the system using Euler:
         u = uAmplitude * Math.cos(2.0 * Math.PI * uFreq * t);
         input.set(0, 0, u);

         for (int i = 0; i < numberOfIntegrationSteps; i++)
         {
            double xNext = x + xDot * eulerStepSize;
            double xDotNext = xDot + (-springK * x - damperB * xDot) * eulerStepSize + u * eulerStepSize;
            t = t + eulerStepSize;

            x = xNext;
            xDot = xDotNext;
         }

         // Simulate the discrete system one step:

         stateDiscreteSimple = computeNextState(stateDiscreteSimple, input, simpleDiscreteA, simpleDiscreteB);
         stateDiscrete = computeNextState(stateDiscrete, input, discreteA, discreteB);
      }

      if (DISPLAY_GRAPHS_AND_SLEEP_FOREVER)
      {
         JFreeChart chart = plot("Test", time, xContinuous, xDiscreteSimple, xDiscrete);

         JFrame jFrame = new JFrame();

         ChartPanel chartPanel = new ChartPanel(chart);
         chartPanel.setPreferredSize(new java.awt.Dimension(1000, 600));
         jFrame.setContentPane(chartPanel);

         jFrame.pack();
         jFrame.setSize(1000, 600);
         jFrame.setVisible(true);
      }

      // Measure sum of square errors between the signals.
      double squaredErrorDiscrete = 0.0;
      double squaredErrorDiscreteSimple = 0.0;

      for (int i = 0; i < xContinuous.length; i++)
      {
         squaredErrorDiscrete = squaredErrorDiscrete + MathTools.square(xContinuous[i] - xDiscrete[i]);
         squaredErrorDiscreteSimple = squaredErrorDiscreteSimple + MathTools.square(xContinuous[i] - xDiscreteSimple[i]);
      }

      squaredErrorDiscrete = squaredErrorDiscrete / ((double) xContinuous.length);
      squaredErrorDiscreteSimple = squaredErrorDiscreteSimple / ((double) xContinuous.length);

      printIfDebug("squaredErrorDiscrete = " + squaredErrorDiscrete);
      printIfDebug("squaredErrorDiscreteSimple = " + squaredErrorDiscreteSimple);

      assertTrue(squaredErrorDiscrete < 1e-4);
      assertTrue(squaredErrorDiscrete * 100.0 < squaredErrorDiscreteSimple);

      assertEquals("Regression Test. Only will be true for certain values. If failing, check changes.", 1.5445857883898253E-5, squaredErrorDiscrete, 1e-7);
      assertEquals("Regression Test. Only will be true for certain values. If failing, check changes.", 0.25136292086153883, squaredErrorDiscreteSimple, 1e-7);

      if (DISPLAY_GRAPHS_AND_SLEEP_FOREVER)
         while(true)
         {
            try
            {
               Thread.sleep(10000);
            }
            catch (InterruptedException e)
            {
            }
         }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testCompareDifferentImplementations()
   {
      int numberOfStates = 30;
      int numberOfInputs = 10;

      StateSpaceSystemDiscretizer singleMatrixExponentialDiscretizer = new SingleMatrixExponentialStateSpaceSystemDiscretizer(numberOfStates, numberOfInputs);
      StateSpaceSystemDiscretizer splitUpMatrixExponentialDiscretizer = new SplitUpMatrixExponentialStateSpaceSystemDiscretizer(numberOfStates, numberOfInputs);
      StateSpaceSystemDiscretizer[] discretizers = new StateSpaceSystemDiscretizer[] { singleMatrixExponentialDiscretizer, splitUpMatrixExponentialDiscretizer };

      Random random = new Random(125L);
      DenseMatrix64F A = RandomMatrices.createRandom(numberOfStates, numberOfStates, random);
      DenseMatrix64F B = RandomMatrices.createRandom(numberOfStates, numberOfInputs, random);
      DenseMatrix64F Q = RandomMatrices.createSymmPosDef(numberOfStates, random);

      DenseMatrix64F[] As = new DenseMatrix64F[] { new DenseMatrix64F(A), new DenseMatrix64F(A) };
      DenseMatrix64F[] Bs = new DenseMatrix64F[] { new DenseMatrix64F(B), new DenseMatrix64F(B) };
      DenseMatrix64F[] Qs = new DenseMatrix64F[] { new DenseMatrix64F(Q), new DenseMatrix64F(Q) };

      double dt = 1.0;

      for (int i = 0; i < discretizers.length; i++)
      {
         discretizers[i].discretize(As[i], Bs[i], Qs[i], dt);
      }

      double tol = 1e-12;
      EjmlUnitTests.assertEquals(As[0], As[1], tol);
      EjmlUnitTests.assertEquals(Bs[0], Bs[1], tol);
      EjmlUnitTests.assertEquals(Qs[0], Qs[1], tol);
   }

   private void printSystemMatrices(String name, DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, double deltaT)
   {
      DenseMatrix64F QDividedByDeltaT = new DenseMatrix64F(Q);
      CommonOps.scale(1.0 / (deltaT), QDividedByDeltaT);

      printIfDebug(name);
      printIfDebug("A = " + A);
      printIfDebug("B = " + B);
      printIfDebug("Scaled Q = " + QDividedByDeltaT);
      printIfDebug("R = " + R);
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

   private JFreeChart plot(String title, double[] time, double[] xContinuous, double[] xDiscreteSimple, double[] xDiscrete)
   {

      XYDataset continuousDataSet = createDataset(time, xContinuous);
      XYDataset discreteSimpleDataset = createDataset(time, xDiscreteSimple);
      XYDataset discreteDataset = createDataset(time, xDiscrete);

      XYItemRenderer renderer1 = new StandardXYItemRenderer();
      NumberAxis rangeAxis1 = new NumberAxis("Continuous");
      XYPlot subplot1 = new XYPlot(continuousDataSet, null, rangeAxis1, renderer1);
      subplot1.setRangeAxisLocation(AxisLocation.BOTTOM_OR_LEFT);
      renderer1.setSeriesVisibleInLegend(0, false);

      XYItemRenderer renderer2 = new StandardXYItemRenderer();
      NumberAxis rangeAxis2 = new NumberAxis("DiscreteSimple");
      XYPlot subplot2 = new XYPlot(discreteSimpleDataset, null, rangeAxis2, renderer2);
      subplot2.setRangeAxisLocation(AxisLocation.TOP_OR_LEFT);
      renderer2.setSeriesVisibleInLegend(0, false);

      XYItemRenderer renderer3 = new StandardXYItemRenderer();
      NumberAxis rangeAxis3 = new NumberAxis("Discrete");
      XYPlot subplot3 = new XYPlot(discreteDataset, null, rangeAxis3, renderer3);
      subplot3.setRangeAxisLocation(AxisLocation.TOP_OR_LEFT);
      renderer3.setSeriesVisibleInLegend(0, false);

      CombinedDomainXYPlot plot = new CombinedDomainXYPlot(); //new LogarithmicAxis("Frequency " + freqUnits));
      //      plot.setGap(10.0);

      plot.add(subplot1, 1);
      plot.add(subplot2, 1);
      plot.add(subplot3, 1);
      plot.setOrientation(PlotOrientation.VERTICAL);

      return new JFreeChart(title, JFreeChart.DEFAULT_TITLE_FONT, plot, true);
   }

   private XYDataset createDataset(double[] xdata, double[] ydata)
   {
      XYSeries series = new XYSeries("data series", false);
      XYSeriesCollection dataset = new XYSeriesCollection();

      for (int i = 0; i < xdata.length; i++)
      {
         series.add(xdata[i], ydata[i]);
      }

      dataset.addSeries(series);

      return dataset;
   }

   private DenseMatrix64F computeNextState(DenseMatrix64F state, DenseMatrix64F input, DenseMatrix64F A, DenseMatrix64F B)
   {
      int numberOfStates = state.getNumRows();

      DenseMatrix64F nextState = new DenseMatrix64F(numberOfStates, 1);

      DenseMatrix64F Ax = new DenseMatrix64F(numberOfStates, 1);
      DenseMatrix64F Bu = new DenseMatrix64F(numberOfStates, 1);

      CommonOps.mult(A, state, Ax);
      CommonOps.mult(B, input, Bu);

      CommonOps.add(Ax, Bu, nextState);

      return nextState;
   }

}
