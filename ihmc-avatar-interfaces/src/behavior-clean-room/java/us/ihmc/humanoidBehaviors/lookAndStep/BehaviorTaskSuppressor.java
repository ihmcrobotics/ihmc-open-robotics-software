package us.ihmc.humanoidBehaviors.lookAndStep;

import org.apache.logging.log4j.Level;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Conditions are evaluated in the order they are added and in the event that a condition
 * returns false, the remaining conditions are NOT evaluated.
 *
 * Condition evaluation should not have side effects because they may not be run.
 */
public class BehaviorTaskSuppressor
{
   private static final Runnable NOOP = () -> {};

   private ConditionHolder currentSuppressionCause;
   private ConditionHolder lastSuppressionCause = new ConditionHolder(); // so first evaluation is always changed
   private long unchangedEvalutionCount = 0;

   private final ArrayList<ConditionHolder> conditions = new ArrayList<>();
   private final StatusLogger statusLogger;
   private final String taskTitle;
   private final Level logLevel;

   private static class ConditionHolder
   {
      Supplier<String> onSuppressMessageSupplier;
      BooleanSupplier evaluateShouldSuppress;
      Runnable onSuppressAction;
   }

   public BehaviorTaskSuppressor(StatusLogger statusLogger, String taskTitle)
   {
      this(statusLogger, taskTitle, Level.INFO);
   }

   public BehaviorTaskSuppressor(StatusLogger statusLogger, String taskTitle, Level logLevel)
   {
      this.statusLogger = statusLogger;
      this.taskTitle = taskTitle;
      this.logLevel = logLevel;
   }

   public void addCondition(String onSuppressMessage, BooleanSupplier evaluateShouldSuppress)
   {
      addCondition(onSuppressMessage, evaluateShouldSuppress, NOOP);
   }

   public void addCondition(String onSuppressMessage, BooleanSupplier evaluateShouldSuppress, Runnable onSuppressAction)
   {
      addCondition(() -> onSuppressMessage, evaluateShouldSuppress, onSuppressAction);
   }

   public void addCondition(Supplier<String> onSuppressMessageSupplier, BooleanSupplier evaluateShouldSuppress)
   {
      addCondition(onSuppressMessageSupplier, evaluateShouldSuppress, NOOP);
   }

   public void addCondition(Supplier<String> onSuppressMessageSupplier, BooleanSupplier evaluateShouldSuppress, Runnable onSuppressAction)
   {
      ConditionHolder conditionHolder = new ConditionHolder();
      conditionHolder.onSuppressMessageSupplier = onSuppressMessageSupplier;
      conditionHolder.evaluateShouldSuppress = evaluateShouldSuppress;
      conditionHolder.onSuppressAction = onSuppressAction;

      conditions.add(conditionHolder);
   }

   public boolean evaluateShouldSuppress()
   {
      return !evaulateShouldAccept();
   }

   public boolean evaulateShouldAccept()
   {
      currentSuppressionCause = null;

      for (ConditionHolder condition : conditions)
      {
         if (condition.evaluateShouldSuppress.getAsBoolean())
         {
            currentSuppressionCause = condition;
            condition.onSuppressAction.run();
            break;
         }
      }

      if (currentSuppressionCause != lastSuppressionCause)
      {
         String message = taskTitle;
         if (currentSuppressionCause != null)
         {
            message += " suppressed. Cause: " +  currentSuppressionCause.onSuppressMessageSupplier.get();
         }
         else
         {
            message += " accepted.";
         }

         if (unchangedEvalutionCount > 0)
         {
            message += " (" + unchangedEvalutionCount + " evaluation";
            if (unchangedEvalutionCount > 1)
            {
               message += "s";
            }
            message += " hidden)";
         }

         statusLogger.log(logLevel, 1, message);

         unchangedEvalutionCount = 0;
      }
      else
      {
         ++unchangedEvalutionCount;
      }

      lastSuppressionCause = currentSuppressionCause;

      return currentSuppressionCause == null;
   }
}
