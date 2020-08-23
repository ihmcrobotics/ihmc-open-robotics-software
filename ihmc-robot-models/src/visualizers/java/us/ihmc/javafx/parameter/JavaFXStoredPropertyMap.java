package us.ihmc.javafx.parameter;

import javafx.scene.control.CheckBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import us.ihmc.log.LogTools;
import us.ihmc.tools.property.StoredPropertyBasics;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.HashMap;

/**
 * It is assumed that all JavaFX controls will be added before
 * any listeners are setup.
 *
 * Things to keep in mind:
 * - Events from JavaFX user will be one at a time
 * - Event from StoredPropertySet updates will be potentially many simulaneous value updates.
 *
 * Therefore, JavaFX user driven inputs can always trigger listeners,
 * but events from StoredPropertySet updates from code can have many different parameters at
 * a time so we need to be careful about callback loops in that situation.
 */
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
      javaFXPropertyHolder.setValue(storedPropertySet.get(booleanKey), false);
   }

   public <T> void put(Spinner<T> doubleSpinner, StoredPropertyKey<T> doubleKey)
   {
      JavaFXPropertyHolder<T> javaFXPropertyHolder = new JavaFXSpinnerPropertyHolder<>(doubleSpinner);
      StoredPropertyBasics<T> storedProperty = storedPropertySet.getProperty(doubleKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);
      javaFXPropertyHolder.setValue(storedPropertySet.get(doubleKey), false);
   }

   public void put(Slider slider, StoredPropertyKey<Double> doubleKey)
   {
      JavaFXPropertyHolder javaFXPropertyHolder = new JavaFXDoubleSliderPropertyHolder(slider);
      StoredPropertyBasics storedProperty = storedPropertySet.getProperty(doubleKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);
      javaFXPropertyHolder.setValue(storedPropertySet.get(doubleKey), false);
   }

   public void putIntegerSlider(Slider slider, StoredPropertyKey<Integer> integerKey)
   {
      JavaFXPropertyHolder javaFXPropertyHolder = new JavaFXIntegerSliderPropertyHolder(slider);
      StoredPropertyBasics storedProperty = storedPropertySet.getProperty(integerKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);
      javaFXPropertyHolder.setValue(storedPropertySet.get(integerKey), false);
   }

   public boolean copyJavaFXToStored()
   {
      boolean anyUpdated = false;
      for (JavaFXPropertyHolder javaFXProperty : toStoredPropertyMap.keySet())
      {
         if (!javaFXProperty.getValue().equals(toStoredPropertyMap.get(javaFXProperty).get()))
         {
            toStoredPropertyMap.get(javaFXProperty).set(javaFXProperty.getValue());
            anyUpdated = true;
         }
      }
      return anyUpdated;
   }

   /**
    * This method won't trigger JavaFX listeners because there could be many updates.
    * Instead, this method returns how many parameters were different so the user
    * can respond if necessary.
    *
    * @return how many parameters were not already equal
    */
   public boolean copyStoredToJavaFX()
   {
      boolean anyUpdated = false;
      for (JavaFXPropertyHolder javaFXProperty : toStoredPropertyMap.keySet())
      {
         if (!javaFXProperty.getValue().equals(toStoredPropertyMap.get(javaFXProperty).get()))
         {
            javaFXProperty.setValue(toStoredPropertyMap.get(javaFXProperty).get(), false);
            anyUpdated = true;
         }
      }
      return anyUpdated;
   }

   public void bindStoredToJavaFXUserInput()
   {
      for (JavaFXPropertyHolder javaFXProperty : toStoredPropertyMap.keySet())
      {
         javaFXProperty.addValueChangedListener(() -> toStoredPropertyMap.get(javaFXProperty).set(javaFXProperty.getValue()));
      }
   }

   public void addAnyJavaFXValueChangedListener(Runnable runnable)
   {
      for (JavaFXPropertyHolder javaFXProperty : toStoredPropertyMap.keySet())
      {
         javaFXProperty.addValueChangedListener(runnable);
      }
   }

   public StoredPropertySetBasics getStoredPropertySet()
   {
      return storedPropertySet;
   }
}
