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

   public void addTrajectorySignal(YoFunctionGeneratorNew functionGenerator, DoubleProvider baselineDesiredValue, OneDoFJointBasics joint)
   {
      trajectorySignals.add(new TrajectorySignal(functionGenerator, joint, baselineDesiredValue, registry));
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
      private final OneDoFJointBasics joint;
      private final YoDouble previousFrequency;
      private final YoInteger counter;
      private final DoubleProvider baselineDesiredValue;

      private long startCount;
      private int controlTicksPerSample;
      private int samplesPerPeriod;
      private int controlTicksPerPeriod;

      private final YoDouble rmsPositionError;
      private final YoDouble rmsVelocityError;

      private final TDoubleArrayList positionErrorsSq = new TDoubleArrayList(new double[MAX_SAMPLES]);
      private final TDoubleArrayList velocityErrorsSq = new TDoubleArrayList(new double[MAX_SAMPLES]);

      TrajectorySignal(YoFunctionGeneratorNew functionGenerator, OneDoFJointBasics joint, DoubleProvider baselineDesiredValue, YoRegistry registry)
      {
         this.functionGenerator = functionGenerator;
         this.joint = joint;
         this.previousFrequency = new YoDouble("prevFreq" + joint.getName(), registry);
         this.baselineDesiredValue = baselineDesiredValue;

         rmsPositionError = new YoDouble("q_err_rms_" + joint.getName(), registry);
         rmsVelocityError = new YoDouble("qd_err_rms_" + joint.getName(), registry);
         counter = new YoInteger("counter" + joint.getName(), registry);
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
            startCount = controllerCounter.getValue();
            counter.set(0);
         }

         if (controlTicksPerSample <= 0)
         {
            return;
         }

         if (counter.getValue() % controlTicksPerSample == 0)
         {
            positionErrorsSq.set(counter.getValue() / controlTicksPerSample, EuclidCoreTools.square(baselineDesiredValue.getValue() - joint.getQ()));
            velocityErrorsSq.set(counter.getValue() / controlTicksPerSample, EuclidCoreTools.square(functionGenerator.getValueDot() - joint.getQd()));
            rmsPositionError.set(Math.sqrt(positionErrorsSq.sum() / samplesPerPeriod));
            rmsVelocityError.set(Math.sqrt(velocityErrorsSq.sum() / samplesPerPeriod));
         }

         counter.set((1 + counter.getValue()) % controlTicksPerPeriod);
      }
   }
}