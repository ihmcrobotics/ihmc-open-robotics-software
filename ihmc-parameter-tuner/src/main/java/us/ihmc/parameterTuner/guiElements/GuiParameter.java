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

   private GuiParameter resetState;
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

   /**
    * This constructor creates a "shallow" copy of the provided parameter. The new parameter
    * will not have a parent and {@link #getParent()} will return {@code null}. However, the
    * unique name of the parameter {@link #getUniqueName()} will include the full namespace
    * of the original parameter.
    */
   public GuiParameter(GuiParameter other)
   {
      super(other.getName(), null, other.getUniqueName());
      this.type = other.getType();
      this.valueOptions = other.getValueOptions();
      set(other);
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

   public GuiParameter createCopy(GuiRegistry parent)
   {
      GuiParameter copy = new GuiParameter(getName(), getType(), getValueOptions(), parent);
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
      if (resetState == null || status != GuiParameterStatus.MODIFIED)
      {
         throw new RuntimeException("Can not reset if the parameter was not modified.");
      }
      set(resetState);
   }

   public void saveStateForReset()
   {
      resetState = createCopy(null);
   }

   @Override
   public String toString()
   {
      String ret = getUniqueName();
      ret += "\ntype: " + type;
      ret += "\nvalue: " + value.getValueSafe();
      return ret;
   }

}
