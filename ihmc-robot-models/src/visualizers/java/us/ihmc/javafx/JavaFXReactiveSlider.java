package us.ihmc.javafx;

import javafx.beans.Observable;
import javafx.scene.control.Slider;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;

import java.util.function.Consumer;

public class JavaFXReactiveSlider
{
   private final Consumer<Number> action;
   private volatile boolean changing = false;

   private static final double MAX_ACTION_FREQUENCY = UnitConversions.hertzToSeconds(4);

   private final Timer throttleTimer = new Timer();
   private final SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

   public JavaFXReactiveSlider(Slider slider, Consumer<Number> action)
   {
      this.action = action;

      slider.valueProperty().addListener(this::valueListener);
      slider.valueChangingProperty().addListener(this::valueChangingListener);
   }

   private void valueListener(Observable observable, Number oldValue, Number newValue)
   {
      executor.queueExecution(() -> waitThenAct(newValue));
   }

   private void waitThenAct(Number newValue)
   {
      throttleTimer.sleepUntilExpiration(MAX_ACTION_FREQUENCY);
      action.accept(newValue);
      throttleTimer.reset();
   }

   private void valueChangingListener(Observable observable, Boolean wasChanging, Boolean isChanging)
   {

   }
}
