package us.ihmc.robotEnvironmentAwareness.ui.properties;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySet;

public abstract class ParametersProperty<T> extends SimpleObjectProperty<T>
{
   private final Map<InvalidationListener, Field> fieldInvalidationListeners = new HashMap<>();
   @SuppressWarnings("rawtypes")
   private final Map<InvalidationListener, EnumField> enumFieldInvalidationListeners = new HashMap<>();

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
      setNumberValue(property, field);
      property.addListener(binding);
      field.addListener(binding);
   }

   protected void bindFieldBidirectionalToConditionalNumberProperty(Condition condition, Property<? extends Number> property, Field field)
   {
      ConditionalNumberBidirectionalBind binding = new ConditionalNumberBidirectionalBind(condition, property, field);
      setNumberValue(property, field);
      property.addListener(binding);
      field.addListener(binding);
   }

   protected void bindFieldBidirectionalToBooleanProperty(Property<Boolean> property, Field field)
   {
      BooleanBidirectionalBind binding = new BooleanBidirectionalBind(property, field);
      setBooleanValue(property, field);
      property.addListener(binding);
      field.addListener(binding);
   }

   protected <E extends Enum<E>> void bindFieldBidirectionalToEnumProperty(Property<E> property, EnumField<E> field)
   {
      EnumBidirectionalBind<E> binding = new EnumBidirectionalBind<>(property, field);
      property.setValue(field.getEnum(getValue()));
      property.addListener(binding);
      field.addListener(binding);
   }

   protected abstract T getValueCopy(T valueToCopy);

   protected abstract class Field implements Observable, NumberGetter<T>, NumberSetter<T>
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

   public interface EnumGetter<E extends Enum<E>, T>
   {
      public E getEnum(T parameters);
   }

   public interface EnumSetter<E extends Enum<E>, T>
   {
      public void setEnum(T parameters, E value);
   }

   protected class DoubleField extends Field implements Observable, NumberGetter<T>, NumberSetter<T>
   {
      public DoubleField(DoubleGetter<T> doubleGetter, DoubleSetter<T> doubleSetter)
      {
         super(doubleGetter, doubleSetter);
      }

      public DoubleField(DoubleStoredPropertyKey key, StoredPropertySet set)
      {
         super(parameters -> set.get(key), (parameters, value) -> set.set(key, value.doubleValue()));
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

   protected class EnumField<E extends Enum<E>> implements Observable, EnumGetter<E, T>, EnumSetter<E, T>
   {
      private EnumGetter<E, T> enumGetter;
      private EnumSetter<E, T> enumSetter;

      public EnumField(EnumGetter<E, T> numberGetter, EnumSetter<E, T> numberSetter)
      {
         this.enumGetter = numberGetter;
         this.enumSetter = numberSetter;
      }

      @Override
      public E getEnum(T parameters)
      {
         return enumGetter.getEnum(parameters);
      }

      @Override
      public void setEnum(T parameters, E value)
      {
         enumSetter.setEnum(parameters, value);
      }

      @Override
      public void addListener(InvalidationListener listener)
      {
         enumFieldInvalidationListeners.put(listener, this);
      }

      @Override
      public void removeListener(InvalidationListener listener)
      {
         enumFieldInvalidationListeners.remove(listener);
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

   protected interface Condition
   {
      boolean checkCondition();
   }

   private class NumberBidirectionalBind extends ConditionalNumberBidirectionalBind
   {
      private NumberBidirectionalBind(Property<? extends Number> numberProperty, Field field)
      {
         super(() -> true, numberProperty, field);
      }
   }

   private class ConditionalNumberBidirectionalBind implements InvalidationListener
   {
      private final Condition condition;
      private final Property<? extends Number> numberProperty;
      private final ParametersProperty<T>.Field field;

      private ConditionalNumberBidirectionalBind(Condition condition, Property<? extends Number> numberProperty, Field field)
      {
         this.condition = condition;
         this.numberProperty = numberProperty;
         this.field = field;
      }

      @Override
      public void invalidated(Observable observable)
      {
         if (!condition.checkCondition())
            return;

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
            setNumberValue(numberProperty, field);
         }
      }
   }

   private class EnumBidirectionalBind<E extends Enum<E>> implements InvalidationListener
   {
      private final Property<E> enumProperty;
      private final ParametersProperty<T>.EnumField<E> field;

      private EnumBidirectionalBind(Property<E> numberProperty, EnumField<E> field)
      {
         this.enumProperty = numberProperty;
         this.field = field;
      }

      @Override
      public void invalidated(Observable observable)
      {

         if (enumProperty.getValue() == field.getEnum(getValue()))
            return;

         if (observable == enumProperty)
         {
            T newParameters = getValueCopy(getValue());
            field.setEnum(newParameters, enumProperty.getValue());
            set(newParameters);
         }
         else
         {
            enumProperty.setValue(field.getEnum(getValue()));
         }
      }
   }

   @SuppressWarnings("unchecked")
   private void setNumberValue(Property<? extends Number> numberProperty, Field field)
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

   private void setBooleanValue(Property<Boolean> booleanProperty, Field field)
   {
      booleanProperty.setValue(getNumberAsBoolean(field.getNumber(getValue())));
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

   @SuppressWarnings("rawtypes")
   @Override
   protected void fireValueChangedEvent()
   {
      super.fireValueChangedEvent();

      for (Entry<InvalidationListener, Field> entry : fieldInvalidationListeners.entrySet())
      {
         entry.getKey().invalidated(entry.getValue());
      }

      for (Entry<InvalidationListener, EnumField> entry : enumFieldInvalidationListeners.entrySet())
      {
         entry.getKey().invalidated(entry.getValue());
      }
   }
}