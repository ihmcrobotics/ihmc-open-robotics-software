package us.ihmc.sensorProcessing.diagnostic;

import java.util.logging.Logger;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class DoubleYoVariableValidityChecker implements DiagnosticUpdatable
{
   private static final int N_TICKS_BEFORE_WARNING = 10;
   private static final int N_TICKS_BEFORE_SEVERE = 100;

   private Logger logger = null;

   private final String inputName;

   private final YoVariableRegistry registry;

   private final YoDouble input;
   private final YoDouble inputPrevious;
   private final YoBoolean hasBeenCalled;

   private final YoBoolean isVariableDead;
   private final YoInteger hasBeenDeadForNTicks;

   private final YoBoolean isVariableNaN;
   private final YoInteger hasBeenNaNForNTicks;

   private final YoBoolean isVariableInfinite;
   private final YoInteger hasBeenInfiniteForNTicks;

   private final YoBoolean enabled;

   private final YoBoolean cannotBeTrusted;

   public DoubleYoVariableValidityChecker(String inputName, YoVariableRegistry parentRegistry)
   {
      this(inputName, null, parentRegistry);
   }

   public DoubleYoVariableValidityChecker(YoDouble input, YoVariableRegistry parentRegistry)
   {
      this(input.getName(), input, parentRegistry);
   }

   private DoubleYoVariableValidityChecker(String inputName, YoDouble input, YoVariableRegistry parentRegistry)
   {
      this.input = input;
      this.inputName = inputName;

      registry = new YoVariableRegistry(inputName + "ValidityChecker");
      parentRegistry.addChild(registry);

      inputPrevious = new YoDouble(inputName + "Previous", registry);
      hasBeenCalled = new YoBoolean(inputName + "ValidityCheckerHasBeenCalled", registry);

      isVariableDead = new YoBoolean(inputName + "IsDead", registry);
      isVariableDead.set(false);
      hasBeenDeadForNTicks = new YoInteger(inputName + "HasBeenDeadForNTicks", registry);

      isVariableNaN = new YoBoolean(inputName + "IsNaN", registry);
      isVariableNaN.set(false);
      hasBeenNaNForNTicks = new YoInteger(inputName + "HasBeenNaNForNTicks", registry);

      isVariableInfinite = new YoBoolean(inputName + "IsInfinite", registry);
      isVariableInfinite.set(false);
      hasBeenInfiniteForNTicks = new YoInteger(inputName + "HasBeenInfiniteForNTicks", registry);

      enabled = new YoBoolean(registry.getName() + "Enabled", registry);

      cannotBeTrusted = new YoBoolean(inputName + "CannotBeTrusted", registry);
   }

   public void setupForLogging(String loggerName)
   {
      if (loggerName != null && !loggerName.isEmpty())
         logger = Logger.getLogger(loggerName);
      else
         logger = Logger.getLogger(getClass().getName());
   }

   @Override
   public void enable()
   {
      enabled.set(true);
   }

   @Override
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
