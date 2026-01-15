package us.ihmc.robotics.math.functionGenerator;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.ArrayList;
import java.util.List;

/**
 * Computes root-mean-squared error over a trajectory period.
 */
public class FunctionGeneratorErrorCalculator
{
   private static final int MAX_SAMPLES = 1000;
   private final double controlDT;
   private final YoLong controllerCounter;

   private final YoRegistry registry;
   private final List<TrajectorySignal> trajectorySignals = new ArrayList<>();

   public FunctionGeneratorErrorCalculator(String namePrefix, double controlDT, YoRegistry registry)
   {
      this.registry = registry;
      this.controlDT = controlDT;
      controllerCounter = new YoLong(namePrefix + "controllerCounter", registry);
   }

   public void addTrajectorySignal(String signalPrefix,
                                   YoFunctionGeneratorNew functionGenerator,
                                   DoubleProvider desiredValue,
                                   DoubleProvider desiredValueDot,
                                   DoubleProvider measuredValue,
                                   DoubleProvider measuredValueDot)
   {
      trajectorySignals.add(new TrajectorySignal(signalPrefix, functionGenerator, desiredValue, desiredValueDot, measuredValue, measuredValueDot, registry));
   }

   public void update()
   {
      controllerCounter.increment();

      for (int i = 0; i < trajectorySignals.size(); i++)
      {
         trajectorySignals.get(i).update();
      }
   }

   private class TrajectorySignal
   {
      private final YoFunctionGeneratorNew functionGenerator;
      private final YoDouble previousFrequency;
      private final YoInteger counter;
      private final DoubleProvider desiredValue, desiredValueDot;
      private final DoubleProvider measuredValue, measuredValueDot;

      private int controlTicksPerSample;
      private int samplesPerPeriod;
      private int controlTicksPerPeriod;

      private final YoDouble rmsPositionError;
      private final YoDouble rmsVelocityError;

      private final TDoubleArrayList positionErrorsSq = new TDoubleArrayList(new double[MAX_SAMPLES]);
      private final TDoubleArrayList velocityErrorsSq = new TDoubleArrayList(new double[MAX_SAMPLES]);

      TrajectorySignal(String signalPrefix,
                       YoFunctionGeneratorNew functionGenerator,
                       DoubleProvider desiredValue,
                       DoubleProvider desiredValueDot,
                       DoubleProvider measuredValue,
                       DoubleProvider measuredValueDot,
                       YoRegistry registry)
      {
         this.functionGenerator = functionGenerator;
         this.previousFrequency = new YoDouble(signalPrefix + "_freq_prev", registry);
         this.desiredValue = desiredValue;
         this.desiredValueDot = desiredValueDot;
         this.measuredValue = measuredValue;
         this.measuredValueDot = measuredValueDot;

         rmsPositionError = new YoDouble(signalPrefix + "_err_rms", registry);
         rmsVelocityError = new YoDouble(signalPrefix + "_d_err_rms", registry);
         counter = new YoInteger(signalPrefix + "_counter", registry);
         previousFrequency.setToNaN();
      }

      void update()
      {
         if (functionGenerator.getMode() == YoFunctionGeneratorMode.OFF || functionGenerator.getFrequency() < 1e-3)
         {
            rmsPositionError.set(0.0);
            rmsVelocityError.set(0.0);
            return;
         }

         if (!EuclidCoreTools.epsilonEquals(functionGenerator.getFrequency(), previousFrequency.getValue(), 1e-5))
         {
            previousFrequency.set(functionGenerator.getFrequency());

            double periodDuration = 1.0 / functionGenerator.getFrequency();
            controlTicksPerPeriod = (int) (periodDuration / controlDT);
            controlTicksPerSample = ((int) Math.ceil((double) controlTicksPerPeriod / MAX_SAMPLES));
            samplesPerPeriod = controlTicksPerPeriod / controlTicksPerSample;

            positionErrorsSq.fill(0.0);
            velocityErrorsSq.fill(0.0);
            counter.set(0);
         }

         if (controlTicksPerSample <= 0)
         {
            return;
         }

         if (counter.getValue() % controlTicksPerSample == 0)
         {
            positionErrorsSq.set(counter.getValue() / controlTicksPerSample, EuclidCoreTools.square(desiredValue.getValue() - measuredValue.getValue()));
            velocityErrorsSq.set(counter.getValue() / controlTicksPerSample, EuclidCoreTools.square(desiredValueDot.getValue() - measuredValueDot.getValue()));
            rmsPositionError.set(Math.sqrt(positionErrorsSq.sum() / samplesPerPeriod));
            rmsVelocityError.set(Math.sqrt(velocityErrorsSq.sum() / samplesPerPeriod));
         }

         counter.set((1 + counter.getValue()) % controlTicksPerPeriod);
      }
   }
}