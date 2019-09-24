package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.Property;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import us.ihmc.log.LogTools;
import us.ihmc.tools.property.StoredPropertyBasics;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.HashMap;

public class JavaFXStoredPropertyMap
{
   private final StoredPropertySetBasics storedPropertySet;
   private final HashMap<JavaFXPropertyHolder, StoredPropertyBasics> map = new HashMap<>();

   public JavaFXStoredPropertyMap(StoredPropertySetBasics storedPropertySet)
   {
      this.storedPropertySet = storedPropertySet;
   }

   public void put(CheckBox checkBox, StoredPropertyKey<Boolean> booleanKey)
   {
      map.put(new JavaFXPropertyHolder<>(() -> checkBox.selectedProperty().getValue(),
                                         runnable -> checkBox.selectedProperty().addListener(observable -> runnable.run())),
              storedPropertySet.getProperty(booleanKey));
      checkBox.setSelected(storedPropertySet.get(booleanKey));
   }

   public <T> void put(Spinner<T> doubleSpinner, StoredPropertyKey<T> doubleKey)
   {
      map.put(new JavaFXPropertyHolder<>(() -> doubleSpinner.getValueFactory().valueProperty().getValue(),
                                         runnable -> doubleSpinner.getValueFactory().valueProperty().addListener(observable -> runnable.run())),
              storedPropertySet.getProperty(doubleKey));
      // TODO should set value here?
   }

   public void put(Slider slider, StoredPropertyKey<Double> doubleKey)
   {
      map.put(new JavaFXPropertyHolder<>(slider::getValue,
                                         runnable -> slider.valueChangingProperty().addListener(
                                               (observable, wasChanging, isChanging) -> {
                                                  if (wasChanging)
                                                  {
                                                     runnable.run();
                                                  }
                                               })),
              storedPropertySet.getProperty(doubleKey));
      slider.setValue(storedPropertySet.get(doubleKey));
   }


   public void put(DoubleProperty valueProperty, StoredPropertyKey<Double> doubleKey)
   {
      map.put(new JavaFXPropertyHolder<>(() -> doubleSpinner.getValueFactory().valueProperty().getValue(),
                                         runnable -> doubleSpinner.getValueFactory().valueProperty().addListener(observable -> runnable.run())),
              storedPropertySet.getProperty(doubleKey));
      map.put(valueProperty, storedPropertySet.getProperty(doubleKey));

      valueProperty.setValue(storedPropertySet.get(doubleKey));
   }
         // add more as needed

   public void copyJavaFXToStored()
   {
      for (Property javaFXProperty : map.keySet())
      {
         map.get(javaFXProperty).set(javaFXProperty.getValue());
      }
   }

   public void copyStoredToJavaFX()
   {
      for (Property javaFXProperty : map.keySet())
      {
         javaFXProperty.setValue(map.get(javaFXProperty).get());
      }
   }

   public void bindStoredToJavaFXUserInput()
   {
      for (Property javaFXProperty : map.keySet())
      {
         javaFXProperty.addListener(observable -> map.get(javaFXProperty).set(javaFXProperty.getValue()));
      }
   }

   public void bindToJavaFXUserInput(Runnable runnable)
   {
      for (Property javaFXProperty : map.keySet())
      {
         javaFXProperty.addListener((observable, wasChanging, isChanging) -> runnable.run());
      }
   }
}
