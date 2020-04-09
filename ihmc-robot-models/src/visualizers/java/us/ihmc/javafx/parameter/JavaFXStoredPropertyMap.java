package us.ihmc.javafx.parameter;

import javafx.scene.control.CheckBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
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
      JavaFXPropertyHolder<Boolean> javaFXPropertyHolder = new JavaFXPropertyHolder<>(() -> checkBox.selectedProperty().getValue(),
                                                                                      value -> checkBox.selectedProperty().setValue(value),
                                                                                      runnable -> checkBox.selectedProperty()
                                                                                                          .addListener(observable -> runnable.run()));
      StoredPropertyBasics<Boolean> storedProperty = storedPropertySet.getProperty(booleanKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);

      checkBox.setSelected(storedPropertySet.get(booleanKey));
   }

   public <T> void put(Spinner<T> doubleSpinner, StoredPropertyKey<T> doubleKey)
   {
      JavaFXPropertyHolder<T> javaFXPropertyHolder = new JavaFXPropertyHolder<>(() -> doubleSpinner.getValueFactory().valueProperty().getValue(),
                                                                                value -> doubleSpinner.getValueFactory().valueProperty().setValue(value),
                                                                                runnable -> doubleSpinner.getValueFactory()
                                                                                                         .valueProperty()
                                                                                                         .addListener(observable -> runnable.run()));
      StoredPropertyBasics<T> storedProperty = storedPropertySet.getProperty(doubleKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);

      doubleSpinner.getValueFactory().setValue(storedPropertySet.get(doubleKey));
   }

   public void put(Slider slider, StoredPropertyKey<Double> doubleKey)
   {
      AtomicBoolean changing = new AtomicBoolean(false); // unfortunately this is necessary for both click and drag to work
      JavaFXPropertyHolder<Double> javaFXPropertyHolder = new JavaFXPropertyHolder<>(slider::getValue,
                                                                                     value -> slider.valueProperty().setValue(value),
                                                                                     runnable ->
                                                                                     {
                                                                                        slider.valueProperty().addListener((observable, oldValue, newValue) ->
                                                                                                                           {
                                                                                                                              if (!changing.get())
                                                                                                                              {
                                                                                                                                 runnable.run();
                                                                                                                              }
                                                                                                                           });
                                                                                        slider.valueChangingProperty()
                                                                                              .addListener((observable, wasChanging, isChanging) ->
                                                                                                           {
                                                                                                              changing.set(isChanging);
                                                                                                              if (wasChanging)
                                                                                                              {
                                                                                                                 runnable.run();
                                                                                                              }
                                                                                                           });
                                                                                     });
      StoredPropertyBasics<Double> storedProperty = storedPropertySet.getProperty(doubleKey);
      toStoredPropertyMap.put(javaFXPropertyHolder, storedProperty);
      fromStoredPropertyMap.put(storedProperty, javaFXPropertyHolder);

      slider.setValue(storedPropertySet.get(doubleKey));
   }

   public void putIntegerSlider(Slider slider, StoredPropertyKey<Integer> integerKey)
   {
      AtomicBoolean changing = new AtomicBoolean(false); // unfortunately this is necessary for both click and drag to work
      JavaFXPropertyHolder<Integer> javaFXPropertyHolder = new JavaFXPropertyHolder<>(() -> (int) slider.getValue(),
                                                                                      value -> slider.valueProperty().setValue(value),
                                                                                      runnable ->
                                                                                      {
                                                                                         slider.valueProperty().addListener((observable, oldValue, newValue) ->
                                                                                                                            {
                                                                                                                               if (!changing.get())
                                                                                                                               {
                                                                                                                                  runnable.run();
                                                                                                                               }
                                                                                                                            });
                                                                                         slider.valueChangingProperty()
                                                                                               .addListener((observable, wasChanging, isChanging) ->
                                                                                                            {
                                                                                                               changing.set(isChanging);
                                                                                                               if (wasChanging)
                                                                                                               {
                                                                                                                  runnable.run();
                                                                                                               }
                                                                                                            });
                                                                                      });

      StoredPropertyBasics<Integer> storedProperty = storedPropertySet.getProperty(integerKey);
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
      javaFXProperty.addValueChangedListener(runnable);
   }

   public StoredPropertySetBasics getStoredPropertySet()
   {
      return storedPropertySet;
   }
}
