package us.ihmc.robotics.linearDynamicSystems;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotics.dataStructures.Polynomial;

class TransferFunctionToStateSpaceConverterTest
{

   @Test
   void testTransferFunctionToStateSpaceConverter()
   {
      boolean plotResponse = false;

      Polynomial numeratorPolynomial = new Polynomial(1.0);
      Polynomial denominatorPolynomial = new Polynomial(1.0, 1.0);
      checkTransferFunctionRemainsTheSame(numeratorPolynomial, denominatorPolynomial);

      double wn = 2.0 * Math.PI;
      double zeta = 0.1;
      numeratorPolynomial = new Polynomial(wn * wn);
      denominatorPolynomial = new Polynomial(1.0, 2.0 * zeta * wn, wn * wn);
      LinearDynamicSystem dynamicSystem = checkTransferFunctionRemainsTheSame(numeratorPolynomial, denominatorPolynomial);

      double[] initialConditions = new double[] {1.0, 0.0};
      if (plotResponse)
         simulateAndPlot(denominatorPolynomial, dynamicSystem, initialConditions);

      numeratorPolynomial = new Polynomial(1.2, 0.3, 1.07);
      denominatorPolynomial = new Polynomial(2.2, 3.3, 4.7, 9.6);
      dynamicSystem = checkTransferFunctionRemainsTheSame(numeratorPolynomial, denominatorPolynomial);

      initialConditions = new double[] {1.0, 0.0, 0.0};
      if (plotResponse)
         simulateAndPlot(denominatorPolynomial, dynamicSystem, initialConditions);

      numeratorPolynomial = new Polynomial(1.2, 0.3, 1.07, 3.3);
      denominatorPolynomial = new Polynomial(2.2, 3.3, 4.7, 9.6);
      dynamicSystem = checkTransferFunctionRemainsTheSame(numeratorPolynomial, denominatorPolynomial);

      initialConditions = new double[] {1.0, 0.0, 0.0};
      if (plotResponse)
         simulateAndPlot(denominatorPolynomial, dynamicSystem, initialConditions);

      if (plotResponse)
         ThreadTools.sleepForever();
   }

   private LinearDynamicSystem checkTransferFunctionRemainsTheSame(Polynomial numeratorPolynomial, Polynomial denominatorPolynomial)
   {
      TransferFunction transferFunction = new TransferFunction(numeratorPolynomial, denominatorPolynomial);
      LinearDynamicSystem dynamicSystem = TransferFunctionToStateSpaceConverter.convertTransferFunctionToStateSpaceObservableCanonicalForm(transferFunction);
      checkTransferFunctionsRemainTheSame(transferFunction, dynamicSystem);

      LinearDynamicSystem dynamicSystemTwo = TransferFunctionToStateSpaceConverter.convertTransferFunctionToStateSpaceControllableCanonicalForm(transferFunction);
      checkTransferFunctionsRemainTheSame(transferFunction, dynamicSystemTwo);

      return dynamicSystem;
   }

   private void checkTransferFunctionsRemainTheSame(TransferFunction transferFunction, LinearDynamicSystem dynamicSystem)
   {
      TransferFunctionMatrix transferFunctionMatrix = dynamicSystem.getTransferFunctionMatrix();
      assertEquals(1, transferFunctionMatrix.getRows());
      assertEquals(1, transferFunctionMatrix.getColumns());

      TransferFunction transferFunctionTwo = transferFunctionMatrix.get(0, 0);
      assertTrue(transferFunction.epsilonEquals(transferFunctionTwo, 1.0e-7));
   }

   private void simulateAndPlot(Polynomial denominatorPolynomial, LinearDynamicSystem dynamicSystem, double[] initialConditions)
   {
      if (dynamicSystem.getOrder() != initialConditions.length)
      {
         throw new RuntimeException("dynamicSystem.getOrder() != initialConditions.length");
      }

      double stepSize = 0.001;
      int numTicks = 5000;
      double[][] stateVersusTime = dynamicSystem.simulateInitialConditions(initialConditions, stepSize, numTicks);

      double[] time = new double[numTicks];
      for (int i = 0; i < numTicks; i++)
      {
         time[i] = stepSize * i;
      }

      double[] x = new double[numTicks];
      for (int i = 0; i < numTicks; i++)
      {
         x[i] = stateVersusTime[i][0];
      }

      MatlabChart fig = new MatlabChart(); // figure('Position',[100 100 640 480]);
      fig.plot(time, x, "-r", 2.0f, "x"); // plot(x,y1,'-r','LineWidth',2);
      //      fig.plot(time, y2, ":k", 3.0f, "BAC"); // plot(x,y2,':k','LineWidth',3);
      fig.RenderPlot(); // First render plot before modifying
      fig.title("Dynamic System Response"); // title('Stock 1 vs. Stock 2');
      //      fig.xlim(10, 100); // xlim([10 100]);
      //      fig.ylim(200, 300); // ylim([200 300]);
      fig.xlabel("t"); // xlabel('Days');
      fig.ylabel("x"); // ylabel('Price');
      fig.grid("on", "on"); // grid on;
      fig.legend("northeast"); // legend('AAPL','BAC','Location','northeast')
      fig.font("Helvetica", 15); // .. 'FontName','Helvetica','FontSize',15

      fig.displayInJFrame();

      if (dynamicSystem.getOrder() >= 2)
      {
         double[] xDot = new double[numTicks];
         for (int i = 0; i < numTicks; i++)
         {
            xDot[i] = stateVersusTime[i][1];
         }

         MatlabChart phasePortrait = new MatlabChart(); // figure('Position',[100 100 640 480]);
         phasePortrait.plot(x, xDot, "-r", 2.0f, "x vs. xDot"); // plot(x,y1,'-r','LineWidth',2);
         //      fig.plot(time, y2, ":k", 3.0f, "BAC"); // plot(x,y2,':k','LineWidth',3);
         phasePortrait.RenderPlot(); // First render plot before modifying
         phasePortrait.title("Dynamic System Response: Phase Portrait"); // title('Stock 1 vs. Stock 2');
         //      fig.xlim(10, 100); // xlim([10 100]);
         //      fig.ylim(200, 300); // ylim([200 300]);
         phasePortrait.xlabel("x"); // xlabel('Days');
         phasePortrait.ylabel("xDot"); // ylabel('Price');
         phasePortrait.grid("on", "on"); // grid on;
         phasePortrait.legend("northeast"); // legend('AAPL','BAC','Location','northeast')
         phasePortrait.font("Helvetica", 15); // .. 'FontName','Helvetica','FontSize',15

         phasePortrait.displayInJFrame();
      }
   }

}
