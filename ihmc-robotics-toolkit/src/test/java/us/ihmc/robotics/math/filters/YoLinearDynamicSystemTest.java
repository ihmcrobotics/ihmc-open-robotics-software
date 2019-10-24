package us.ihmc.robotics.math.filters;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotics.dataStructures.ComplexNumber;
import us.ihmc.robotics.dataStructures.Polynomial;
import us.ihmc.robotics.linearDynamicSystems.LinearDynamicSystem;
import us.ihmc.robotics.linearDynamicSystems.MatlabChart;
import us.ihmc.robotics.linearDynamicSystems.TransferFunction;
import us.ihmc.robotics.linearDynamicSystems.TransferFunctionToStateSpaceConverter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

class YoLinearDynamicSystemTest
{

   @Test
   void testYoLinearDynamicSystem()
   {
      boolean displayChart = false;

      YoVariableRegistry registry = new YoVariableRegistry("test");

      double zeta = 0.5;
      double wn = 2.0 * Math.PI * 1.0;
      TransferFunction transferFunctionOne = TransferFunction.constructSecondOrderTransferFunction(1.0, wn, zeta);

      double dcScale = 1.0;
      double[] realZeros = new double[] {-4.0};
      ComplexNumber[] complexPairZeros = null;
      double[] realPoles = new double[] {-2.0};
      ComplexNumber[] complexPairPoles = null;

      Polynomial numeratorPolynomialTwo = Polynomial.constructFromScaleFactorAndRoots(dcScale, realZeros, complexPairZeros);
      Polynomial denominatorPolynomialTwo = Polynomial.constructFromScaleFactorAndRoots(0.25, realPoles, complexPairPoles);
      TransferFunction transferFunctionTwo = new TransferFunction(numeratorPolynomialTwo, denominatorPolynomialTwo);

      TransferFunction transferFunction = transferFunctionOne.times(transferFunctionTwo);
      LinearDynamicSystem linearDynamicSystem = TransferFunctionToStateSpaceConverter.convertTransferFunctionToStateSpaceControllableCanonicalForm(transferFunction);

      assertEquals(transferFunction.getOrder(), linearDynamicSystem.getOrder());
      assertEquals(1, linearDynamicSystem.getInputSize());
      assertEquals(1, linearDynamicSystem.getOutputSize());

      String statePrefix = "x_";
      String inputPrefix = "u_";
      String outputPrefix = "y_";
      YoLinearDynamicSystem dynamicSystem = new YoLinearDynamicSystem(linearDynamicSystem, statePrefix, inputPrefix, outputPrefix, registry);

      double dt = 0.001;
      double finalTime = 2.0;
      int numberOfSteps = (int) (finalTime / dt);

      double amplitude = 1.0;
      double frequency = 0.25;

      YoDouble time = new YoDouble("time", registry);

      double[] output = new double[numberOfSteps];
      double[] timeArray = new double[numberOfSteps];

      for (int i = 0; i < numberOfSteps; i++)
      {
         timeArray[i] = time.getValue();
         double[] input = new double[] {amplitude * Math.sin(8.0 * Math.PI * frequency * time.getDoubleValue())};
         dynamicSystem.update(input, dt);

         output[i] = dynamicSystem.getOutput()[0];
         time.add(dt);
      }

      if (displayChart)
      {
         MatlabChart chart = new MatlabChart();
         chart.plot(timeArray, output, "-r", 2.0f, "YoLinearDynamicSystemTest");
         chart.RenderPlot();
         chart.title("YoLinearDynamicSystemTest");
         chart.xlabel("t");
         chart.ylabel("x");
         chart.grid("on", "on");
         chart.displayInJFrame();

         ThreadTools.sleepForever();
      }
   }

}
