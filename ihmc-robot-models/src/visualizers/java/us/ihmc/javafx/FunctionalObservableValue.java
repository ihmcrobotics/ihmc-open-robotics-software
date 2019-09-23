package us.ihmc.javafx;

import javafx.beans.InvalidationListener;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;

import java.util.ArrayList;
import java.util.function.Supplier;

public class FunctionalObservableValue<T> implements ObservableValue<T>
{
   private final ArrayList<ChangeListener> changeListeners = new ArrayList<>();
   private final ArrayList<InvalidationListener> invalidationListeners = new ArrayList<>();
   private final Supplier<T> getter;

   public FunctionalObservableValue(Supplier<T> getter)
   {
      this.getter = getter;
   }

   @Override
   public T getValue()
   {
      return getter.get();
   }

   @Override
   public void addListener(ChangeListener<? super T> listener)
   {
      changeListeners.add(listener);
   }

   @Override
   public void removeListener(ChangeListener<? super T> listener)
   {
      changeListeners.remove(listener);
   }

   @Override
   public void addListener(InvalidationListener listener)
   {
      invalidationListeners.add(listener);
   }

   @Override
   public void removeListener(InvalidationListener listener)
   {
      invalidationListeners.remove(listener);
   }
}
