package us.ihmc.sensorProcessing.diagnostic;

import java.util.logging.Logger;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;

public class YoVariableValidityChecker implements ProcessingYoVariable
{
   private static final int N_TICKS_BEFORE_SEVERE = 100;

   private Logger logger = null;

   private final DoubleYoVariable input;
   private final DoubleYoVariable inputPrevious;
   private final BooleanYoVariable hasBeenCalled;

   private final BooleanYoVariable isVariableDead;
   private final IntegerYoVariable hasBeenDeadForNTicks;

   private final BooleanYoVariable isVariableNaN;
   private final IntegerYoVariable hasBeenNaNForNTicks;

   private final BooleanYoVariable isVariableInfinite;
   private final IntegerYoVariable hasBeenInfiniteForNTicks;

   public YoVariableValidityChecker(DoubleYoVariable input, YoVariableRegistry registry)
   {
      this.input = input;
      inputPrevious = new DoubleYoVariable(input.getName() + "Previous", registry);
      hasBeenCalled = new BooleanYoVariable(input.getName() + "ValidityCheckerHasBeenCalled", registry);

      isVariableDead = new BooleanYoVariable(input.getName() + "IsDead", registry);
      isVariableDead.set(false);
      hasBeenDeadForNTicks = new IntegerYoVariable(input.getName() + "HasBeenDeadForNTicks", registry);

      isVariableNaN = new BooleanYoVariable(input.getName() + "IsNaN", registry);
      isVariableNaN.set(false);
      hasBeenNaNForNTicks = new IntegerYoVariable(input.getName() + "HasBeenNaNForNTicks", registry);

      isVariableInfinite = new BooleanYoVariable(input.getName() + "IsInfinite", registry);
      isVariableInfinite.set(false);
      hasBeenInfiniteForNTicks = new IntegerYoVariable(input.getName() + "HasBeenInfiniteForNTicks", registry);
   }

   public void setupForLogging(String loggerName)
   {
      if (loggerName != null && !loggerName.isEmpty())
         logger = Logger.getLogger(loggerName);
      else
         logger = Logger.getLogger(getClass().getName());
   }

   @Override
   public void update()
   {
      checkIfVariableIsDead();
      checkIfVariableIsNaN();
      checkIfVariableIsInfinite();

      inputPrevious.set(input.getDoubleValue());

      if (!hasBeenCalled.getBooleanValue())
         hasBeenCalled.set(true);
   }

   private void checkIfVariableIsDead()
   {
      boolean isDead;

      if (hasBeenCalled.getBooleanValue())
         isDead = input.getDoubleValue() == inputPrevious.getDoubleValue();
      else
         isDead = false;

      boolean wasDead = isVariableDead.getBooleanValue();

      if (!isDead)
         hasBeenDeadForNTicks.set(0);
      else
         hasBeenDeadForNTicks.increment();

      if (hasBeenDeadForNTicks.getIntegerValue() == 1)
         logger.warning("Input signal " + input.getName() + " might be dead.");
      else if (hasBeenDeadForNTicks.getIntegerValue() == N_TICKS_BEFORE_SEVERE)
         logger.severe("Input signal " + input.getName() + " has been dead for " + Integer.toString(N_TICKS_BEFORE_SEVERE) + " ticks.");
      else if (!isDead && wasDead)
         logger.info("Input signal " + input.getName() + " is coming back to life.");

      isVariableDead.set(isDead);
   }

   private void checkIfVariableIsNaN()
   {
      boolean isNaN = Double.isNaN(input.getDoubleValue());
      boolean wasNaN = isVariableNaN.getBooleanValue();

      if (!isNaN)
         hasBeenNaNForNTicks.set(0);
      else
         hasBeenNaNForNTicks.increment();

      if (hasBeenNaNForNTicks.getIntegerValue() == 1)
         logger.warning("Input signal " + input.getName() + " is NaN.");
      else if (hasBeenNaNForNTicks.getIntegerValue() == N_TICKS_BEFORE_SEVERE)
         logger.severe("Input signal " + input.getName() + " has been NaN for " + Integer.toString(N_TICKS_BEFORE_SEVERE) + " ticks.");
      else if (!isNaN && wasNaN)
         logger.info("Input signal " + input.getName() + " is not NaN anymore.");

      isVariableNaN.set(isNaN);
   }

   private void checkIfVariableIsInfinite()
   {
      boolean isInfinite = Double.isInfinite(input.getDoubleValue());
      boolean wasInfinite = isVariableInfinite.getBooleanValue();

      if (!isInfinite)
         hasBeenInfiniteForNTicks.set(0);
      else
         hasBeenInfiniteForNTicks.increment();

      if (hasBeenInfiniteForNTicks.getIntegerValue() == 1)
         logger.warning("Input signal " + input.getName() + " is infinite.");
      else if (hasBeenInfiniteForNTicks.getIntegerValue() == N_TICKS_BEFORE_SEVERE)
         logger.severe("Input signal " + input.getName() + " has been infinite for " + Integer.toString(N_TICKS_BEFORE_SEVERE) + " ticks.");
      else if (!isInfinite && wasInfinite)
         logger.info("Input signal " + input.getName() + " is not infinite anymore.");

      isVariableNaN.set(isInfinite);
   }
}
