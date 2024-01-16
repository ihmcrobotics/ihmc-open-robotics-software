package us.ihmc.robotics.math.functionGenerator;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorNew;
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
   private static final int SAMPLES_PER_PERIOD = 100;
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

   public void addTrajectorySignal(YoFunctionGeneratorNew functionGenerator, OneDoFJointBasics joint)
   {
      trajectorySignals.add(new TrajectorySignal(functionGenerator, joint, registry));
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
      private final YoInteger controlTicksPerSample;
      private final YoLong startCount;
      private final YoInteger counter;

      private final YoDouble rmsPositionError;
      private final YoDouble rmsVelocityError;

      private final TDoubleArrayList positionErrorsSq = new TDoubleArrayList(new double[SAMPLES_PER_PERIOD]);
      private final TDoubleArrayList velocityErrorsSq = new TDoubleArrayList(new double[SAMPLES_PER_PERIOD]);
      private boolean firstTick = true;

      TrajectorySignal(YoFunctionGeneratorNew functionGenerator, OneDoFJointBasics joint, YoRegistry registry)
      {
         this.functionGenerator = functionGenerator;
         this.joint = joint;
         this.previousFrequency = new YoDouble("prevFreq" + joint.getName(), registry);
         this.controlTicksPerSample = new YoInteger("sampleFreq" + joint.getName(), registry);

         rmsPositionError = new YoDouble("q_err_rms_" + joint.getName(), registry);
         rmsVelocityError = new YoDouble("qd_err_rms_" + joint.getName(), registry);

         startCount = new YoLong("startCount" + joint.getName(), registry);
         counter = new YoInteger("counter" + joint.getName(), registry);
      }

      void update()
      {
         if (firstTick || !EuclidCoreTools.epsilonEquals(functionGenerator.getFrequency(), previousFrequency.getValue(), 1e-5))
         {
            firstTick = false;
            previousFrequency.set(functionGenerator.getFrequency());
            double periodDuration = 1.0 / functionGenerator.getFrequency();
            double sampleDT = periodDuration / SAMPLES_PER_PERIOD;
            controlTicksPerSample.set(Math.max((int) (sampleDT / controlDT), 1));

            positionErrorsSq.fill(0.0);
            velocityErrorsSq.fill(0.0);

            startCount.set(controllerCounter.getValue());
            counter.set(0);
         }

         long count = controllerCounter.getValue() - startCount.getValue();
         if (controlTicksPerSample.getValue() > 0 && count % controlTicksPerSample.getValue() == 0)
         {
            positionErrorsSq.set(counter.getValue(), EuclidCoreTools.square(functionGenerator.getValue() - joint.getQ()));
            velocityErrorsSq.set(counter.getValue(), EuclidCoreTools.square(functionGenerator.getValueDot() - joint.getQd()));

            rmsPositionError.set(Math.sqrt(positionErrorsSq.sum() / SAMPLES_PER_PERIOD));
            rmsVelocityError.set(Math.sqrt(velocityErrorsSq.sum() / SAMPLES_PER_PERIOD));

            counter.set((counter.getValue() + 1) % SAMPLES_PER_PERIOD);
         }
      }
   }
}
