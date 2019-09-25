package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.scene.control.CheckBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
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
                                         value -> checkBox.selectedProperty().setValue(value),
                                         runnable -> checkBox.selectedProperty().addListener(observable -> runnable.run())),
              storedPropertySet.getProperty(booleanKey));
      checkBox.setSelected(storedPropertySet.get(booleanKey));
   }

   public <T> void put(Spinner<T> doubleSpinner, StoredPropertyKey<T> doubleKey)
   {
      map.put(new JavaFXPropertyHolder<>(() -> doubleSpinner.getValueFactory().valueProperty().getValue(),
                                         value -> doubleSpinner.getValueFactory().valueProperty().setValue(value),
                                         runnable -> doubleSpinner.getValueFactory().valueProperty().addListener(observable -> runnable.run())),
              storedPropertySet.getProperty(doubleKey));
      // TODO should set value here?
   }

   public void put(Slider slider, StoredPropertyKey<Double> doubleKey)
   {
      map.put(new JavaFXPropertyHolder<>(slider::getValue,
                                         value -> slider.valueProperty().setValue(value),
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

   public void putIntegerSlider(Slider slider, StoredPropertyKey<Integer> integerKey)
   {
      map.put(new JavaFXPropertyHolder<>(() -> (int) slider.getValue(),
                                         value -> slider.valueProperty().setValue(value),
                                         runnable -> slider.valueChangingProperty().addListener(
                                               (observable, wasChanging, isChanging) -> {
                                                  if (wasChanging)
                                                  {
                                                     runnable.run();
                                                  }
                                               })),
              storedPropertySet.getProperty(integerKey));
      slider.setValue(storedPropertySet.get(integerKey));
   }

   public void copyJavaFXToStored()
   {
      for (JavaFXPropertyHolder javaFXProperty : map.keySet())
      {
         map.get(javaFXProperty).set(javaFXProperty.getValue());
      }
   }

   public void copyStoredToJavaFX()
   {
      for (JavaFXPropertyHolder javaFXProperty : map.keySet())
      {
         javaFXProperty.setValue(map.get(javaFXProperty).get());
      }
   }

   public void bindStoredToJavaFXUserInput()
   {
      for (JavaFXPropertyHolder javaFXProperty : map.keySet())
      {
         javaFXProperty.addValueChangedListener(() -> map.get(javaFXProperty).set(javaFXProperty.getValue()));
      }
   }

   public void bindToJavaFXUserInput(Runnable runnable)
   {
      for (JavaFXPropertyHolder javaFXProperty : map.keySet())
      {
         javaFXProperty.addValueChangedListener(runnable);
      }
   }
}
