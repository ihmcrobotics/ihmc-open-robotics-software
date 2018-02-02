package us.ihmc.parameterTuner.guiElements;

import gnu.trove.map.TObjectIntMap;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;

public class GuiParameter extends GuiElement
{
   private final String type;
   private final TObjectIntMap<String> valueOptions;

   private final StringProperty value = new SimpleStringProperty();
   private final StringProperty min = new SimpleStringProperty();
   private final StringProperty max = new SimpleStringProperty();
   private final StringProperty description = new SimpleStringProperty();

   public GuiParameter(String name, String type, GuiRegistry parent)
   {
      this(name, type, null, parent);
   }

   public GuiParameter(String name, String type, TObjectIntMap<String> valueOptions, GuiRegistry parent)
   {
      super(name, parent);
      this.type = type;
      this.valueOptions = valueOptions;
   }

   public void setValue(String value)
   {
      this.value.set(value);
   }

   public void setMin(String min)
   {
      this.min.set(min);
   }

   public void setMax(String max)
   {
      this.max.set(max);
   }

   public void setDescription(String description)
   {
      this.description.set(description);
   }

   public String getType()
   {
      return type;
   }

   public TObjectIntMap<String> getValueOptions()
   {
      return valueOptions;
   }

   public String getCurrentValue()
   {
      return value.get();
   }

   public String getCurrentMin()
   {
      return min.get();
   }

   public String getCurrentMax()
   {
      return max.get();
   }

   public String getCurrentDescription()
   {
      return description.get();
   }

   public void addChangedListener(ParameterChangeListener listener)
   {
      value.addListener((observable, oldValue, newValue) -> listener.changed(this));
      min.addListener((observable, oldValue, newValue) -> listener.changed(this));
      max.addListener((observable, oldValue, newValue) -> listener.changed(this));
      description.addListener((observable, oldValue, newValue) -> listener.changed(this));
   }

   public GuiParameter createCopy()
   {
      GuiParameter copy = new GuiParameter(getName(), getType(), getValueOptions(), getParent());
      copy.set(this);
      return copy;
   }

   public void set(GuiParameter other)
   {
      setValue(other.getCurrentValue());
      setMin(other.getCurrentMin());
      setMax(other.getCurrentMax());
      setDescription(other.getCurrentDescription());
   }
}
