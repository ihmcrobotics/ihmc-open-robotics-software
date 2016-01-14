package us.ihmc.sensorProcessing.diagnostic;

import java.util.logging.Logger;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public class DoubleYoVariableValidityChecker implements DiagnosticUpdatable
{
   private static final int N_TICKS_BEFORE_WARNING = 10;
   private static final int N_TICKS_BEFORE_SEVERE = 100;

   private Logger logger = null;

   private final String inputName;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable input;
   private final DoubleYoVariable inputPrevious;
   private final BooleanYoVariable hasBeenCalled;

   private final BooleanYoVariable isVariableDead;
   private final IntegerYoVariable hasBeenDeadForNTicks;

   private final BooleanYoVariable isVariableNaN;
   private final IntegerYoVariable hasBeenNaNForNTicks;

   private final BooleanYoVariable isVariableInfinite;
   private final IntegerYoVariable hasBeenInfiniteForNTicks;

   private final BooleanYoVariable enabled;

   private final BooleanYoVariable cannotBeTrusted;

   public DoubleYoVariableValidityChecker(String inputName, YoVariableRegistry parentRegistry)
   {
      this(inputName, null, parentRegistry);
   }

   public DoubleYoVariableValidityChecker(DoubleYoVariable input, YoVariableRegistry parentRegistry)
   {
      this(input.getName(), input, parentRegistry);
   }

   private DoubleYoVariableValidityChecker(String inputName, DoubleYoVariable input, YoVariableRegistry parentRegistry)
   {
      this.input = input;
      this.inputName = inputName;

      registry = new YoVariableRegistry(inputName + "ValidityChecker");
      parentRegistry.addChild(registry);

      inputPrevious = new DoubleYoVariable(inputName + "Previous", registry);
      hasBeenCalled = new BooleanYoVariable(inputName + "ValidityCheckerHasBeenCalled", registry);

      isVariableDead = new BooleanYoVariable(inputName + "IsDead", registry);
      isVariableDead.set(false);
      hasBeenDeadForNTicks = new IntegerYoVariable(inputName + "HasBeenDeadForNTicks", registry);

      isVariableNaN = new BooleanYoVariable(inputName + "IsNaN", registry);
      isVariableNaN.set(false);
      hasBeenNaNForNTicks = new IntegerYoVariable(inputName + "HasBeenNaNForNTicks", registry);

      isVariableInfinite = new BooleanYoVariable(inputName + "IsInfinite", registry);
      isVariableInfinite.set(false);
      hasBeenInfiniteForNTicks = new IntegerYoVariable(inputName + "HasBeenInfiniteForNTicks", registry);

      enabled = new BooleanYoVariable(registry.getName() + "Enabled", registry);

      cannotBeTrusted = new BooleanYoVariable(inputName + "CannotBeTrusted", registry);
   }

   public void setupForLogging(String loggerName)
   {
      if (loggerName != null && !loggerName.isEmpty())
         logger = Logger.getLogger(loggerName);
      else
         logger = Logger.getLogger(getClass().getName());
   }

   public void enable()
   {
      enabled.set(true);
   }

   public void disable()
   {
      reset();
      enabled.set(false);
   }

   @Override
   public void update()
   {
      if (input == null)
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "input variable to call update(), otherwise use update(double)");
      update(input.getDoubleValue());
   }

   public void update(double newInputValue)
   {
      if (!enabled.getBooleanValue())
         return;

      checkIfVariableIsDead(newInputValue);
      checkIfVariableIsNaN(newInputValue);
      checkIfVariableIsInfinite(newInputValue);
      
      inputPrevious.set(newInputValue);
      
      if (!hasBeenCalled.getBooleanValue())
         hasBeenCalled.set(true);

      boolean hasBeenDeadForALongTime = hasBeenDeadForNTicks.getIntegerValue() >= N_TICKS_BEFORE_SEVERE;
      boolean hasBeenInfiniteForALongTime = hasBeenInfiniteForNTicks.getIntegerValue() >= N_TICKS_BEFORE_SEVERE;
      boolean hasBeenNaNForALongTime = hasBeenNaNForNTicks.getIntegerValue() >= N_TICKS_BEFORE_SEVERE;
      cannotBeTrusted.set(hasBeenDeadForALongTime || hasBeenInfiniteForALongTime || hasBeenNaNForALongTime);
   }

   private void checkIfVariableIsDead(double newInputValue)
   {
      boolean isDead;

      if (hasBeenCalled.getBooleanValue())
         isDead = newInputValue == inputPrevious.getDoubleValue();
      else
         isDead = false;

      boolean wasDead = isVariableDead.getBooleanValue();

      int previousDeadNTicks = hasBeenDeadForNTicks.getIntegerValue();

      if (!isDead)
         hasBeenDeadForNTicks.set(0);
      else
         hasBeenDeadForNTicks.increment();

      if (logger != null)
      {
         if (hasBeenDeadForNTicks.getIntegerValue() == N_TICKS_BEFORE_WARNING)
            logger.warning("Input signal " + inputName + " might be dead.");
         else if (hasBeenDeadForNTicks.getIntegerValue() == N_TICKS_BEFORE_SEVERE)
            logger.severe("Input signal " + inputName + " has been dead for " + Integer.toString(N_TICKS_BEFORE_SEVERE) + " ticks.");
         else if (!isDead && wasDead && previousDeadNTicks >= N_TICKS_BEFORE_WARNING)
            logger.info("Input signal " + inputName + " is coming back to life.");
      }

      isVariableDead.set(isDead);
   }

   private void checkIfVariableIsNaN(double newInputValue)
   {
      boolean isNaN = Double.isNaN(newInputValue);
      boolean wasNaN = isVariableNaN.getBooleanValue();

      int previousNaNNTicks = hasBeenNaNForNTicks.getIntegerValue();

      if (!isNaN)
         hasBeenNaNForNTicks.set(0);
      else
         hasBeenNaNForNTicks.increment();

      if (logger != null)
      {
         if (hasBeenNaNForNTicks.getIntegerValue() == N_TICKS_BEFORE_WARNING)
            logger.warning("Input signal " + inputName + " is NaN.");
         else if (hasBeenNaNForNTicks.getIntegerValue() == N_TICKS_BEFORE_SEVERE)
            logger.severe("Input signal " + inputName + " has been NaN for " + Integer.toString(N_TICKS_BEFORE_SEVERE) + " ticks.");
         else if (!isNaN && wasNaN && previousNaNNTicks >= N_TICKS_BEFORE_WARNING)
            logger.info("Input signal " + inputName + " is not NaN anymore.");
      }

      isVariableNaN.set(isNaN);
   }

   private void checkIfVariableIsInfinite(double newInputValue)
   {
      boolean isInfinite = Double.isInfinite(newInputValue);
      boolean wasInfinite = isVariableInfinite.getBooleanValue();

      int previousInfiniteNTicks = hasBeenInfiniteForNTicks.getIntegerValue();

      if (!isInfinite)
         hasBeenInfiniteForNTicks.set(0);
      else
         hasBeenInfiniteForNTicks.increment();

      if (logger != null)
      {
         if (hasBeenInfiniteForNTicks.getIntegerValue() == N_TICKS_BEFORE_WARNING)
            logger.warning("Input signal " + inputName + " is infinite.");
         else if (hasBeenInfiniteForNTicks.getIntegerValue() == N_TICKS_BEFORE_SEVERE)
            logger.severe("Input signal " + inputName + " has been infinite for " + Integer.toString(N_TICKS_BEFORE_SEVERE) + " ticks.");
         else if (!isInfinite && wasInfinite && previousInfiniteNTicks >= N_TICKS_BEFORE_WARNING)
            logger.info("Input signal " + inputName + " is not infinite anymore.");
      }

      isVariableNaN.set(isInfinite);
   }

   public void reset()
   {
      isVariableDead.set(false);
      isVariableInfinite.set(false);
      isVariableNaN.set(false);

      hasBeenDeadForNTicks.set(0);
      hasBeenInfiniteForNTicks.set(0);
      hasBeenNaNForNTicks.set(0);

      cannotBeTrusted.set(false);
   }

   public boolean isInputSane()
   {
      return !isVariableInfinite.getBooleanValue() && !isVariableNaN.getBooleanValue();
   }

   public boolean isInputAlive()
   {
      return !isVariableDead.getBooleanValue();
   }

   public boolean variableCannotBeTrusted()
   {
      return cannotBeTrusted.getBooleanValue();
   }
}
