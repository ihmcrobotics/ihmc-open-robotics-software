package us.ihmc.robotEnvironmentAwareness.ui.properties;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;

public abstract class ParametersProperty<T> extends SimpleObjectProperty<T>
{
   private final Map<InvalidationListener, Field> fieldInvalidationListeners = new HashMap<>();

   public ParametersProperty()
   {
      super();
   }

   public ParametersProperty(T initialValue)
   {
      super(initialValue);
   }

   public ParametersProperty(Object bean, String name)
   {
      super(bean, name);
   }

   public ParametersProperty(Object bean, String name, T initialValue)
   {
      super(bean, name, initialValue);
   }

   protected void bindFieldBidirectionalToNumberProperty(Property<? extends Number> property, Field field)
   {
      NumberBidirectionalBind binding = new NumberBidirectionalBind(property, field);
      property.addListener(binding);
      field.addListener(binding);
   }
   
   protected void bindFieldBidirectionalToBooleanProperty(Property<Boolean> property, Field field)
   {
      BooleanBidirectionalBind binding = new BooleanBidirectionalBind(property, field);
      property.addListener(binding);
      field.addListener(binding);
   }

   protected abstract T getValueCopy(T valueToCopy);

   private abstract class Field implements Observable, NumberGetter<T>, NumberSetter<T>
   {
      private NumberGetter<T> numberGetter;
      private NumberSetter<T> numberSetter;

      public Field(NumberGetter<T> numberGetter, NumberSetter<T> numberSetter)
      {
         this.numberGetter = numberGetter;
         this.numberSetter = numberSetter;
      }

      @Override
      public Number getNumber(T parameters)
      {
         return numberGetter.getNumber(parameters);
      }

      @Override
      public void setNumber(T parameters, Number value)
      {
         numberSetter.setNumber(parameters, value);
      }

      @Override
      public void addListener(InvalidationListener listener)
      {
         fieldInvalidationListeners.put(listener, this);
      }

      @Override
      public void removeListener(InvalidationListener listener)
      {
         fieldInvalidationListeners.remove(listener);
      }
   }

   private interface NumberGetter<T>
   {
      public Number getNumber(T parameters);
   }

   private interface NumberSetter<T>
   {
      public void setNumber(T parameters, Number value);
   }

   protected class DoubleField extends Field implements Observable, NumberGetter<T>, NumberSetter<T>
   {
      public DoubleField(DoubleGetter<T> doubleGetter, DoubleSetter<T> doubleSetter)
      {
         super(doubleGetter, doubleSetter);
      }
   }

   protected interface DoubleGetter<T> extends NumberGetter<T>
   {
      public double get(T parameters);

      @Override
      public default Number getNumber(T parameters)
      {
         return new Double(get(parameters));
      }
   }

   protected interface DoubleSetter<T> extends NumberSetter<T>
   {
      public void set(T parameters, double value);

      @Override
      public default void setNumber(T parameters, Number value)
      {
         set(parameters, value.doubleValue());
      }
   }

   protected class FloatField extends Field implements Observable, NumberGetter<T>, NumberSetter<T>
   {
      public FloatField(FloatGetter<T> floatGetter, FloatSetter<T> floatSetter)
      {
         super(floatGetter, floatSetter);
      }
   }

   protected interface FloatGetter<T> extends NumberGetter<T>
   {
      public float get(T parameters);

      @Override
      public default Number getNumber(T parameters)
      {
         return new Float(get(parameters));
      }
   }

   protected interface FloatSetter<T> extends NumberSetter<T>
   {
      public void set(T parameters, float value);

      @Override
      public default void setNumber(T parameters, Number value)
      {
         set(parameters, value.floatValue());
      }
   }

   protected class IntegerField extends Field implements Observable, NumberGetter<T>, NumberSetter<T>
   {
      public IntegerField(IntegerGetter<T> integerGetter, IntegerSetter<T> integerSetter)
      {
         super(integerGetter, integerSetter);
      }
   }

   protected interface IntegerGetter<T> extends NumberGetter<T>
   {
      public int get(T parameters);

      @Override
      public default Number getNumber(T parameters)
      {
         return new Integer(get(parameters));
      }
   }

   protected interface IntegerSetter<T> extends NumberSetter<T>
   {
      public void set(T parameters, int value);

      @Override
      public default void setNumber(T parameters, Number value)
      {
         set(parameters, value.intValue());
      }
   }

   protected class BooleanField extends Field implements Observable, NumberGetter<T>, NumberSetter<T>
   {
      public BooleanField(BooleanGetter<T> booleanGetter, BooleanSetter<T> booleanSetter)
      {
         super(booleanGetter, booleanSetter);
      }
   }

   protected interface BooleanGetter<T> extends NumberGetter<T>
   {
      public boolean get(T parameters);

      @Override
      public default Number getNumber(T parameters)
      {
         return getBooleanAsNumber(get(parameters));
      }
   }

   protected interface BooleanSetter<T> extends NumberSetter<T>
   {
      public void set(T parameters, boolean value);

      @Override
      public default void setNumber(T parameters, Number value)
      {
         set(parameters, getNumberAsBoolean(value));
      }
   }

   private class NumberBidirectionalBind implements InvalidationListener
   {
      private final Property<? extends Number> numberProperty;
      private final ParametersProperty<T>.Field field;

      private NumberBidirectionalBind(Property<? extends Number> numberProperty, Field field)
      {
         this.numberProperty = numberProperty;
         this.field = field;
      }

      @SuppressWarnings("unchecked")
      @Override
      public void invalidated(Observable observable)
      {
         if (numberProperty.getValue().doubleValue() == field.getNumber(getValue()).doubleValue())
            return;

         if (observable == numberProperty)
         {
            T newParameters = getValueCopy(getValue());
            field.setNumber(newParameters, numberProperty.getValue());
            set(newParameters);
         }
         else
         {
            if (numberProperty.getValue() instanceof Double)
               ((Property<Double>) numberProperty).setValue(new Double(field.getNumber(getValue()).doubleValue()));
            else if (numberProperty.getValue() instanceof Integer)
               ((Property<Integer>) numberProperty).setValue(new Integer(field.getNumber(getValue()).intValue()));
            else if (numberProperty.getValue() instanceof Float)
               ((Property<Float>) numberProperty).setValue(new Float(field.getNumber(getValue()).floatValue()));
            else if (numberProperty.getValue() instanceof Long)
               ((Property<Long>) numberProperty).setValue(new Long(field.getNumber(getValue()).longValue()));
            else if (numberProperty.getValue() instanceof Short)
               ((Property<Short>) numberProperty).setValue(new Short(field.getNumber(getValue()).shortValue()));
            else if (numberProperty.getValue() instanceof Byte)
               ((Property<Byte>) numberProperty).setValue(new Byte(field.getNumber(getValue()).byteValue()));
            else
               throw new RuntimeException("Unhandled instance of Number: " + numberProperty.getValue().getClass().getSimpleName());
         }
      }
   }

   private class BooleanBidirectionalBind implements InvalidationListener
   {
      private final Property<Boolean> booleanProperty;
      private final ParametersProperty<T>.Field field;

      private BooleanBidirectionalBind(Property<Boolean> booleanProperty, Field field)
      {
         this.booleanProperty = booleanProperty;
         this.field = field;
      }

      @Override
      public void invalidated(Observable observable)
      {
         if (booleanProperty.getValue().booleanValue() == getNumberAsBoolean(field.getNumber(getValue())))
            return;

         if (observable == booleanProperty)
         {
            T newParameters = getValueCopy(getValue());
            field.setNumber(newParameters, getBooleanAsNumber(booleanProperty.getValue()));
            set(newParameters);
         }
         else
         {
            booleanProperty.setValue(getNumberAsBoolean(field.getNumber(getValue())));
         }
      }
   }

   private static Number getBooleanAsNumber(Boolean bool)
   {
      return bool ? 1 : 0;
   }

   private static boolean getNumberAsBoolean(Number number)
   {
      int intValue = number.intValue();
      if (intValue == 1)
         return true;
      else if (intValue == 0)
         return false;
      else
         throw new RuntimeException("Unhandled number to set as boolean: " + intValue);
   }

   @Override
   protected void fireValueChangedEvent()
   {
      super.fireValueChangedEvent();

      for (Entry<InvalidationListener, Field> entry : fieldInvalidationListeners.entrySet())
      {
         entry.getKey().invalidated(entry.getValue());
      }
   }
}