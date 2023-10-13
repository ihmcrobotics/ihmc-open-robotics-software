package us.ihmc.behaviors.lookAndStep;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static us.ihmc.behaviors.lookAndStep.BehaviorTaskSuppressor.NOOP;

/**
 * A suppression condition for input to a module in the subsumption style
 * architecture.
 */
public class SuppressionCondition
{
   private final Supplier<String> onSuppressMessageSupplier;
   private final BooleanSupplier evaluateShouldSuppress;
   private final Runnable onSuppressAction;

   public SuppressionCondition(String onSuppressMessage, BooleanSupplier evaluateShouldSuppress)
   {
      this(onSuppressMessage, evaluateShouldSuppress, NOOP);
   }

   public SuppressionCondition(String onSuppressMessage, BooleanSupplier evaluateShouldSuppress, Runnable onSuppressAction)
   {
      this(() -> onSuppressMessage, evaluateShouldSuppress, onSuppressAction);
   }

   public SuppressionCondition(Supplier<String> onSuppressMessageSupplier, BooleanSupplier evaluateShouldSuppress)
   {
      this(onSuppressMessageSupplier, evaluateShouldSuppress, NOOP);
   }

   public SuppressionCondition(Supplier<String> onSuppressMessageSupplier, BooleanSupplier evaluateShouldSuppress, Runnable onSuppressAction)
   {
      this.onSuppressMessageSupplier = onSuppressMessageSupplier;
      this.evaluateShouldSuppress = evaluateShouldSuppress;
      this.onSuppressAction = onSuppressAction;
   }

   public Supplier<String> getOnSuppressMessageSupplier()
   {
      return onSuppressMessageSupplier;
   }

   public BooleanSupplier getEvaluateShouldSuppress()
   {
      return evaluateShouldSuppress;
   }

   public Runnable getOnSuppressAction()
   {
      return onSuppressAction;
   }
}
