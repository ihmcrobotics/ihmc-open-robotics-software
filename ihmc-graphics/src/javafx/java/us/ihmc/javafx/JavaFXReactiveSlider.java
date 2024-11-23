package us.ihmc.javafx;

import javafx.application.Platform;
import javafx.beans.Observable;
import javafx.scene.control.Slider;
import us.ihmc.commons.MathTools;
import us.ihmc.tools.Timer;
import us.ihmc.commons.UnitConversions;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.function.Consumer;

public class JavaFXReactiveSlider
{
   private static final double MAX_ACTION_FREQUENCY = UnitConversions.hertzToSeconds(4);

   private final Slider slider;
   private final Consumer<Number> action;
   private final Timer throttleTimer = new Timer();
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   private boolean skipNextChange = false; // prevent feedback loo

   public JavaFXReactiveSlider(Slider slider, Consumer<Number> action)
   {
      this.slider = slider;
      this.action = action;

      slider.valueProperty().addListener(this::valueListener);
   }

   private void valueListener(Observable observable, Number oldValue, Number newValue)
   {
      if (skipNextChange)
      {
         skipNextChange = false;
         return;
      }
      executor.clearQueueAndExecute(() -> waitThenAct(newValue));
   }

   private void waitThenAct(Number newValue)
   {
      throttleTimer.sleepUntilExpiration(MAX_ACTION_FREQUENCY);
      action.accept(newValue);
      throttleTimer.reset();
   }

   public void acceptUpdatedValue(double newValue)
   {
      double clampedValue = MathTools.clamp(newValue, 0.0, 100.0);
      Platform.runLater(() ->
      {
         boolean executing = executor.isExecuting();
         boolean valueChanging = slider.isValueChanging();

         if (!executing && !valueChanging)
         {
            skipNextChange = true;
            slider.setValue(clampedValue);
         }
      });
   }
}
