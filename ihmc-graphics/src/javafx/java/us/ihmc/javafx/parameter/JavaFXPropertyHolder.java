package us.ihmc.javafx.parameter;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;

import java.util.ArrayList;
import java.util.Objects;

public abstract class JavaFXPropertyHolder<T>
{
   private final ArrayList<Runnable> changedListeners = new ArrayList<>();
   protected ChangeListener<T> changeListener = this::changed;
   protected ChangeListener<Boolean> changingListener = this::changing;
   private volatile boolean skipNextChange = false;
   private volatile boolean isChanging = false;

   public JavaFXPropertyHolder()
   {

   }

   protected void changed(ObservableValue<? extends T> observable, T oldValue, T newValue)
   {
      if (!skipNextChange && !isChanging && !Objects.equals(oldValue, newValue))
      {
         for (Runnable changedListener : changedListeners)
         {
            changedListener.run();
         }
      }

      skipNextChange = false;
   }

   protected void changing(ObservableValue<? extends Boolean> observable, Boolean wasChanging, Boolean isChanging)
   {
      this.isChanging = isChanging;
      if (wasChanging) // necessary for both click and drag to work for sliders
      {
         for (Runnable changedListener : changedListeners)
         {
            changedListener.run();
         }
      }
   }

   protected void setValue(T value, boolean notifyListeners)
   {
      if (!notifyListeners)
         skipNextChange = true;
      setValueInternal(value);
   }

   public void addValueChangedListener(Runnable onChanged)
   {
      changedListeners.add(onChanged);
   }

   public void setValue(T value)
   {
      setValue(value, true);
   }

   public abstract T getValue();

   protected abstract void setValueInternal(T value);
}
