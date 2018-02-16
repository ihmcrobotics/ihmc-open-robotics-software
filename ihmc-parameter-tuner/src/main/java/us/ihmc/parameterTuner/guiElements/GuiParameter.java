package us.ihmc.parameterTuner.guiElements;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.map.TObjectIntMap;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import us.ihmc.yoVariables.parameters.ParameterLoadStatus;

public class GuiParameter extends GuiElement
{
   private final String type;
   private final TObjectIntMap<String> valueOptions;

   private final StringProperty value = new SimpleStringProperty();
   private final StringProperty min = new SimpleStringProperty();
   private final StringProperty max = new SimpleStringProperty();
   private final StringProperty description = new SimpleStringProperty();

   private GuiParameter initialParameter;
   private GuiParameterStatus status;

   private final List<ParameterChangeListener> listeners = new ArrayList<>();

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

   public void addStatusUpdater()
   {
      // Only consider the parameter modified if the value changes for now.
      value.addListener((observable, oldValue, newValue) -> status = GuiParameterStatus.MODIFIED);
   }

   public void markAsModified()
   {
      if (status != GuiParameterStatus.MODIFIED)
      {
         status = GuiParameterStatus.MODIFIED;
         informListeners();
      }
   }

   public void setValue(String value)
   {
      if (!this.value.getValueSafe().equals(value))
      {
         this.value.set(value);
         informListeners();
      }
   }

   public void setMin(String min)
   {
      if (!this.min.getValueSafe().equals(min))
      {
         this.min.set(min);
         informListeners();
      }
   }

   public void setMax(String max)
   {
      if (!this.max.getValueSafe().equals(max))
      {
         this.max.set(max);
         informListeners();
      }
   }

   public void setDescription(String description)
   {
      if (!this.description.getValueSafe().equals(description))
      {
         this.description.set(description);
         informListeners();
      }
   }

   public void setLoadStatus(ParameterLoadStatus loadStatus)
   {
      setStatus(GuiParameterStatus.get(loadStatus));
   }

   public void setStatus(GuiParameterStatus status)
   {
      if (status != this.status)
      {
         this.status = status;
         informListeners();
      }
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

   public GuiParameterStatus getStatus()
   {
      return status;
   }

   public void addChangedListener(ParameterChangeListener listener)
   {
      listeners.add(listener);
   }

   private void informListeners()
   {
      listeners.stream().forEach(listener -> listener.changed(this));
   }

   public GuiParameter createCopy()
   {
      GuiParameter copy = new GuiParameter(getName(), getType(), getValueOptions(), getParent());
      copy.set(this);
      return copy;
   }

   public void set(GuiParameter other)
   {
      value.set(other.getCurrentValue());
      min.set(other.getCurrentMin());
      max.set(other.getCurrentMax());
      description.set(other.getCurrentDescription());
      status = other.getStatus();
      informListeners();
   }

   public void setValueAndStatus(GuiParameter other)
   {
      value.set(other.getCurrentValue());
      status = other.getStatus();
      informListeners();
   }

   public void reset()
   {
      if (initialParameter == null || status != GuiParameterStatus.MODIFIED)
      {
         throw new RuntimeException("Can not reset if the parameter was not modified.");
      }
      set(initialParameter);
   }

   public void saveStateForReset()
   {
      initialParameter = createCopy();
   }

}
