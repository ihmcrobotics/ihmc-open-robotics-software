package us.ihmc.robotEnvironmentAwareness.ui.properties;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class JavaFXPropertyHolder<T>
{
   private final Supplier<T> valueSupplier;
   private final Consumer<Runnable> registerChangeListener;

   public JavaFXPropertyHolder(Supplier<T> valueSupplier, Consumer<Runnable> registerChangeListener)
   {
      this.valueSupplier = valueSupplier;
      this.registerChangeListener = registerChangeListener;
   }

   public T getValue()
   {
      return valueSupplier.get();
   }

   public void addValueChangedListener(Runnable onChanged)
   {
      registerChangeListener.accept(onChanged);
   }
}
