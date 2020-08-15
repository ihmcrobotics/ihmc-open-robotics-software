package us.ihmc.javafx.parameter;

import javafx.scene.control.CheckBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import us.ihmc.log.LogTools;
import us.ihmc.tools.property.StoredPropertyBasics;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicBoolean;

public class JavaFXStoredPropertyMap
{
   private final StoredPropertySetBasics storedPropertySet;
   private final HashMap<JavaFXPropertyHolder, StoredPropertyBasics> toStoredPropertyMap = new HashMap<>();
   private final HashMap<StoredPropertyBasics, JavaFXPropertyHolder> fromStoredPropertyMap = new HashMap<>();

   public JavaFXStoredPropertyMap(StoredPropertySetBasics storedPropertySet)
   {
      this.storedPropertySet = storedPropertySet;
   }

   public void put(CheckBox checkBox, StoredPropertyKey<Boolean> booleanKey)
   {
      JavaFXPropertyHolder<Boolean> javaFXPropertyHolder = new JavaFXCheckBoxPropertyHolder(checkBox);
      StoredPropertyBasics<Boolean> storedProperty = storedPropertySet.getProperty(booleanKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);

      checkBox.setSelected(storedPropertySet.get(booleanKey));
   }

   public <T> void put(Spinner<T> doubleSpinner, StoredPropertyKey<T> doubleKey)
   {
      JavaFXPropertyHolder<T> javaFXPropertyHolder = new JavaFXSpinnerPropertyHolder<>(doubleSpinner);
      StoredPropertyBasics<T> storedProperty = storedPropertySet.getProperty(doubleKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);

      doubleSpinner.getValueFactory().setValue(storedPropertySet.get(doubleKey));
   }

   public void put(Slider slider, StoredPropertyKey<Double> doubleKey)
   {
      AtomicBoolean changing = new AtomicBoolean(false); // unfortunately this is necessary for both click and drag to work
      JavaFXPropertyHolder javaFXPropertyHolder = new JavaFXDoubleSliderPropertyHolder(slider);
      StoredPropertyBasics storedProperty = storedPropertySet.getProperty(doubleKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);

      slider.setValue(storedPropertySet.get(doubleKey));
   }

   public void putIntegerSlider(Slider slider, StoredPropertyKey<Integer> integerKey)
   {
      AtomicBoolean changing = new AtomicBoolean(false); // unfortunately this is necessary for both click and drag to work
      JavaFXPropertyHolder javaFXPropertyHolder = new JavaFXIntegerSliderPropertyHolder(slider);
      StoredPropertyBasics storedProperty = storedPropertySet.getProperty(integerKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);

      slider.setValue(storedPropertySet.get(integerKey));
   }

   public void copyJavaFXToStored()
   {
      for (JavaFXPropertyHolder javaFXProperty : toStoredPropertyMap.keySet())
      {
         toStoredPropertyMap.get(javaFXProperty).set(javaFXProperty.getValue());
      }
   }

   public void copyStoredToJavaFX()
   {
      for (JavaFXPropertyHolder javaFXProperty : toStoredPropertyMap.keySet())
      {
         javaFXProperty.setValue(toStoredPropertyMap.get(javaFXProperty).get());
      }
   }

   public void bindStoredToJavaFXUserInput()
   {
      for (JavaFXPropertyHolder javaFXProperty : toStoredPropertyMap.keySet())
      {
         javaFXProperty.addValueChangedListener(() -> toStoredPropertyMap.get(javaFXProperty).set(javaFXProperty.getValue()));
      }
   }

   public void bindToJavaFXUserInput(Runnable runnable)
   {
      for (JavaFXPropertyHolder javaFXProperty : toStoredPropertyMap.keySet())
      {
         javaFXProperty.addValueChangedListener(runnable);
      }
   }

   public void bindStoredToJavaFXUserInput(StoredPropertyKey<?> storedPropertyKey)
   {
      StoredPropertyBasics storedProperty = storedPropertySet.getProperty(storedPropertyKey);
      JavaFXPropertyHolder javaFXProperty = fromStoredPropertyMap.get(storedProperty);
      javaFXProperty.addValueChangedListener(() -> storedProperty.set(javaFXProperty.getValue()));
   }

   public void bindToJavaFXUserInput(StoredPropertyKey<?> storedPropertyKey, Runnable runnable)
   {
      StoredPropertyBasics storedProperty = storedPropertySet.getProperty(storedPropertyKey);
      JavaFXPropertyHolder javaFXProperty = fromStoredPropertyMap.get(storedProperty);
      javaFXProperty.addValueChangedListener(() -> {
         LogTools.info("Calling that runnable for {}", storedPropertyKey.getTitleCasedName());
         runnable.run();
      });
   }

   public StoredPropertySetBasics getStoredPropertySet()
   {
      return storedPropertySet;
   }
}
