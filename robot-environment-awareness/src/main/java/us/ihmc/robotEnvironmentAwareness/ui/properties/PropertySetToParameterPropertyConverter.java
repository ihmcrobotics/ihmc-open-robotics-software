package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.InvalidationListener;
import javafx.beans.Observable;
import javafx.beans.property.Property;
import us.ihmc.tools.property.*;

public class PropertySetToParameterPropertyConverter
{


   public void bidirectionalBindToDoubleProperty(Property<? extends Number> property, DoubleStoredPropertyKey key, StoredPropertySetBasics propertySet, InvalidationListener propertyChangedListener)
   {
      DoubleBidirectionalBind binding = new DoubleBidirectionalBind(property, key, propertySet, propertyChangedListener);
      ((Property<Double>) property).setValue(propertySet.get(key));
      property.addListener(binding);
      key.addListener(binding);
   }

   public void bidirectionalBindToIntegerProperty(Property<? extends Number> property, IntegerStoredPropertyKey key, StoredPropertySetBasics propertySet, InvalidationListener propertyChangedListener)
   {
      IntegerBidirectionalBind binding = new IntegerBidirectionalBind(property, key, propertySet, propertyChangedListener);
      ((Property<Integer>) property).setValue(propertySet.get(key));
      property.addListener(binding);
      key.addListener(binding);
   }

   public void bidirectionalBindToBooleanProperty(Property<Boolean> property, BooleanStoredPropertyKey key, StoredPropertySetBasics propertySet, InvalidationListener propertyChangedListener)
   {
      BooleanBidirectionalBind binding = new BooleanBidirectionalBind(property, key, propertySet, propertyChangedListener);
      property.setValue(propertySet.get(key));
      property.addListener(binding);
      key.addListener(binding);
   }

   private class DoubleBidirectionalBind implements InvalidationListener
   {
      private final Property<? extends Number> numberProperty;
      private final DoubleStoredPropertyKey key;
      private final StoredPropertySetBasics propertySet;
      private final InvalidationListener fieldChangeListener;

      private DoubleBidirectionalBind(Property<? extends Number> numberProperty, DoubleStoredPropertyKey key, StoredPropertySetBasics propertySet, InvalidationListener propertyChangedListener)
      {
         this.numberProperty = numberProperty;
         this.key = key;
         this.propertySet = propertySet;
         this.fieldChangeListener = propertyChangedListener;
      }

      @Override
      public void invalidated(Observable observable)
      {
         double propertyValue = propertySet.get(key);

         if (numberProperty.getValue().doubleValue() == propertyValue)
            return;

         if (observable == numberProperty)
         {
             propertySet.set(key, numberProperty.getValue().doubleValue());
             fieldChangeListener.invalidated(observable);
         }
         else
         {
            ((Property<Double>) numberProperty).setValue(propertyValue);
         }
      }
   }

   private class IntegerBidirectionalBind implements InvalidationListener
   {
      private final Property<? extends Number> numberProperty;
      private final IntegerStoredPropertyKey key;
      private final StoredPropertySetBasics propertySet;
      private final InvalidationListener fieldChangeListener;

      private IntegerBidirectionalBind(Property<? extends Number> numberProperty, IntegerStoredPropertyKey key, StoredPropertySetBasics propertySet, InvalidationListener fieldChangeListener)
      {
         this.numberProperty = numberProperty;
         this.key = key;
         this.propertySet = propertySet;
         this.fieldChangeListener = fieldChangeListener;
      }

      @Override
      public void invalidated(Observable observable)
      {
         int propertyValue = propertySet.get(key);

         if (numberProperty.getValue().intValue() == propertyValue)
            return;

         if (observable == numberProperty)
         {
            propertySet.set(key, numberProperty.getValue().intValue());
            fieldChangeListener.invalidated(observable);
         }
         else
         {
            ((Property<Integer>) numberProperty).setValue(propertyValue);
         }
      }
   }

   private class BooleanBidirectionalBind implements InvalidationListener
   {
      private final Property<Boolean> booleanProperty;
      private final BooleanStoredPropertyKey key;
      private final StoredPropertySetBasics propertySet;
      private final InvalidationListener fieldChangeListener;

      private BooleanBidirectionalBind(Property<Boolean> numberProperty, BooleanStoredPropertyKey key, StoredPropertySetBasics propertySet, InvalidationListener fieldChangeListener)
      {
         this.booleanProperty = numberProperty;
         this.key = key;
         this.propertySet = propertySet;
         this.fieldChangeListener = fieldChangeListener;
      }

      @Override
      public void invalidated(Observable observable)
      {
         boolean propertyValue = propertySet.get(key);

         if (booleanProperty.getValue() == propertyValue)
            return;

         if (observable == booleanProperty)
         {
            propertySet.set(key, booleanProperty.getValue());
            fieldChangeListener.invalidated(observable);
         }
         else
         {
            booleanProperty.setValue(propertyValue);
         }
      }
   }
}