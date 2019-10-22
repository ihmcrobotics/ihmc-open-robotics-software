package us.ihmc.robotEnvironmentAwareness.ui.properties;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class JavaFXPropertyHolder<T>
{
   private final Supplier<T> valueSupplier;
   private final Consumer<T> setter;
   private final Consumer<Runnable> registerChangeListener;

   public JavaFXPropertyHolder(Supplier<T> valueSupplier, Consumer<T> setter, Consumer<Runnable> registerChangeListener)
   {
      this.valueSupplier = valueSupplier;
      this.setter = setter;
      this.registerChangeListener = registerChangeListener;
   }

   public T getValue()
   {
      return valueSupplier.get();
   }

   public void setValue(T value)
   {
      setter.accept(value);
   }

   public void addValueChangedListener(Runnable onChanged)
   {
      registerChangeListener.accept(onChanged);
   }
}
