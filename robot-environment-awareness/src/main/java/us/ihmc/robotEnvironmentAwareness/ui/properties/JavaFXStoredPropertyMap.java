package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty.Field;
import us.ihmc.tools.property.StoredProperty;
import us.ihmc.tools.property.StoredPropertyBasics;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.HashMap;

public class JavaFXStoredPropertyMap extends HashMap<Property, StoredPropertyBasics>
{
   private final StoredPropertySetBasics storedPropertySet;

   public JavaFXStoredPropertyMap(StoredPropertySetBasics storedPropertySet)
   {
      this.storedPropertySet = storedPropertySet;
   }

   public void put(CheckBox checkBox, StoredPropertyKey<Boolean> booleanKey)
   {
      put(checkBox.selectedProperty(), storedPropertySet.getProperty(booleanKey));
      checkBox.setSelected(storedPropertySet.get(booleanKey));
   }

   public <T> void put(Spinner<T> doubleSpinner, StoredPropertyKey<T> doubleKey)
   {
      put(doubleSpinner.getValueFactory().valueProperty(), doubleKey);
   }

   public <T> void put(ObjectProperty<T> property, StoredPropertyKey<T> key)
   {
      put(property, storedPropertySet.getProperty(key));
      property.setValue(storedPropertySet.get(key));
   }

   public <T> void put(Property<T> slider, StoredPropertyKey<T> doubleKey)
   {
      put(slider, storedPropertySet.getProperty(doubleKey));
      slider.setValue(storedPropertySet.get(doubleKey));
   }

   public void put(Slider slider, StoredPropertyKey<Integer> doubleKey)
   {
      put(slider.valueProperty(), storedPropertySet.getProperty(doubleKey));
      slider.setValue(storedPropertySet.get(doubleKey));
   }


   public void put(DoubleProperty valueProperty, StoredPropertyKey<Double> doubleKey)
   {
      put(valueProperty, storedPropertySet.getProperty(doubleKey));

      valueProperty.setValue(storedPropertySet.get(doubleKey));
   }
         // add more as needed

   public void copyJavaFXToStored()
   {
      for (Property javaFXProperty : keySet())
      {
         get(javaFXProperty).set(javaFXProperty.getValue());
      }
   }

   public void copyStoredToJavaFX()
   {
      for (Property javaFXProperty : keySet())
      {
         javaFXProperty.setValue(get(javaFXProperty).get());
      }
   }

   public void bindStoredToJavaFXUserInput()
   {
      for (Property javaFXProperty : keySet())
      {
         javaFXProperty.addListener(observable -> get(javaFXProperty).set(javaFXProperty.getValue()));
      }
   }

   public void bindToJavaFXUserInput(Runnable runnable)
   {
      for (Property javaFXProperty : keySet())
      {
         javaFXProperty.addListener(observable -> runnable.run());
      }
   }
}
