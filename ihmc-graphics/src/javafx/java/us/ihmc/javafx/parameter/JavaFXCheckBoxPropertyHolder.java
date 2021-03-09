package us.ihmc.javafx.parameter;

import javafx.scene.control.CheckBox;

public class JavaFXCheckBoxPropertyHolder extends JavaFXPropertyHolder<Boolean>
{
   private final CheckBox checkBox;

   public JavaFXCheckBoxPropertyHolder(CheckBox checkBox)
   {
      this.checkBox = checkBox;

      checkBox.selectedProperty().addListener(changeListener);
   }

   @Override
   public Boolean getValue()
   {
      return checkBox.selectedProperty().getValue();
   }

   @Override
   protected void setValueInternal(Boolean value)
   {
      checkBox.selectedProperty().setValue(value);
   }
}
