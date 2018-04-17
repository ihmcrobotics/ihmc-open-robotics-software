package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.File;

import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

public abstract class REABasicUIController
{
   protected REAUIMessager uiMessager;
   private FilePropertyHelper filePropertyHelper;

   public abstract void bindControls();

   public REABasicUIController()
   {
   }

   public void attachREAMessager(REAUIMessager uiMessager)
   {
      this.uiMessager = uiMessager;
   }

   public void setConfigurationFile(File configurationFile)
   {
      filePropertyHelper = new FilePropertyHelper(configurationFile);
   }

   protected void saveUIControlProperty(Topic<Boolean> topic, ToggleButton toggleButton)
   {
      filePropertyHelper.saveProperty(topic.getName(), toggleButton.isSelected());
   }

   protected <T> void saveUIControlProperty(Topic<T> topic, ComboBox<T> comboBox)
   {
      filePropertyHelper.saveProperty(topic.getName(), comboBox.getValue().toString());
   }

   protected void saveUIControlProperty(Topic<? extends Number> topic, Slider slider)
   {
      filePropertyHelper.saveProperty(topic.getName(), slider.getValue());
   }

   protected void loadUIControlProperty(Topic<Boolean> topic, ToggleButton toggleButton)
   {
      Boolean loadedProperty = filePropertyHelper.loadBooleanProperty(topic.getName());
      if (loadedProperty != null)
         toggleButton.setSelected(loadedProperty);
   }

   protected void loadUIControlProperty(Topic<? extends Number> topic, Slider slider)
   {
      Double loadedProperty = filePropertyHelper.loadDoubleProperty(topic.getName());
      if (loadedProperty != null)
         slider.setValue(loadedProperty);
   }

   protected void loadUIControlProperty(Topic<Double> topic, Spinner<Double> spinner)
   {
      Double loadedProperty = filePropertyHelper.loadDoubleProperty(topic.getName());
      if (loadedProperty != null)
         spinner.getValueFactory().setValue(loadedProperty);
   }

   @SuppressWarnings("unchecked")
   protected <T extends Enum<T>> void loadUIControlProperty(Topic<T> topic, ComboBox<T> comboBox)
   {
      String loadedProperty = filePropertyHelper.loadProperty(topic.getName());
      if (loadedProperty != null)
      {
         T valueOf = (T) Enum.valueOf(comboBox.getItems().get(0).getClass(), loadedProperty);
         comboBox.setValue(valueOf);
      }
   }
}