package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
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
   }

   public void put(Spinner<Double> doubleSpinner, StoredPropertyKey<Double> doubleKey)
   {
      put(doubleSpinner.getValueFactory().valueProperty(), storedPropertySet.getProperty(doubleKey));
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
