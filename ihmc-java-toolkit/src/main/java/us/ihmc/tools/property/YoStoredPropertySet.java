package us.ihmc.tools.property;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Provides a load/saveable property set accessed by strongly typed static keys.
 *
 * The property INI file is saved to the classpath by file and loaded from the classpath by resource.
 *
 * Some of the benefits of this framework:
 * - Keys are created with title cased names available for GUI fields
 * - No YoVariableServer required
 * - INI file can be placed in higher level projects to override the defaults
 */
public class YoStoredPropertySet implements StoredPropertySetBasics
{
   private final StoredPropertyKeyList keys;

   private final HashMap<StoredPropertyKey, YoVariable<?>> yoVariables = new HashMap<>();

   public YoStoredPropertySet(StoredPropertyKeyList keys, YoVariableRegistry registry)
   {
      this.keys = keys;


      for (StoredPropertyKey<?> key : keys.keys())
      {
         if (key.getType() == Double.class)
            yoVariables.put(key, new YoDouble(key.getCamelCasedName(), registry));
         else if (key.getType() == Integer.class)
            yoVariables.put(key, new YoInteger(key.getCamelCasedName(), registry));
         else if (key.getType() == Boolean.class)
            yoVariables.put(key, new YoBoolean(key.getCamelCasedName(), registry));
         else
            throw new RuntimeException("Type is not yet implemented.");
      }
   }

   @Override
   public double get(DoubleStoredPropertyKey key)
   {
      return ((YoDouble) yoVariables.get(key)).getDoubleValue();
   }

   @Override
   public int get(IntegerStoredPropertyKey key)
   {
      return ((YoInteger) yoVariables.get(key)).getIntegerValue();
   }

   @Override
   public boolean get(BooleanStoredPropertyKey key)
   {
      return ((YoBoolean) yoVariables.get(key)).getBooleanValue();
   }

   @Override
   public <T> T get(StoredPropertyKey<T> key)
   {
      return null;
   }

   @Override
   public void set(DoubleStoredPropertyKey key, double value)
   {
      if (get(key) == value)
         return;

      ((YoDouble) yoVariables.get(key)).set(value);
      key.notifyOfVariableChanged();
   }

   @Override
   public void set(IntegerStoredPropertyKey key, int value)
   {
      if (get(key) == value)
         return;

      ((YoInteger) yoVariables.get(key)).set(value);
      key.notifyOfVariableChanged();
   }

   @Override
   public void set(BooleanStoredPropertyKey key, boolean value)
   {
      if (get(key) == value)
         return;

      ((YoBoolean) yoVariables.get(key)).set(value);
      key.notifyOfVariableChanged();
   }

   @Override
   public <T> void set(StoredPropertyKey<T> key, T value)
   {

   }

   @Override
   public <T> StoredPropertyBasics<T> getProperty(StoredPropertyKey<T> key)
   {
      return null;
   }

   @Override
   public List<Object> getAll()
   {
      List<Object> values = new ArrayList<>();
      for (int i = 0; i < keys.keys().size(); i++)
      {
         if (keys.keys().get(i).getType() == Double.class)
            values.add(((YoDouble) yoVariables.get(keys.keys().get(i))).getDoubleValue());
         else if (keys.keys().get(i).getType() == Integer.class)
            values.add(((YoInteger) yoVariables.get(keys.keys().get(i))).getIntegerValue());
         else if (keys.keys().get(i).getType() == Boolean.class)
            values.add(((YoBoolean) yoVariables.get(keys.keys().get(i))).getBooleanValue());
      }
      return values;
   }

   @Override
   public void setAll(List<Object> newValues)
   {
      for (int i = 0; i < keys.keys().size(); i++)
      {
         if (keys.keys().get(i).getType() == Double.class)
            ((YoDouble) yoVariables.get(keys.keys().get(i))).set((Double) newValues.get(i));
         else if (keys.keys().get(i).getType() == Integer.class)
            ((YoInteger) yoVariables.get(keys.keys().get(i))).set((Integer) newValues.get(i));
         else if (keys.keys().get(i).getType() == Boolean.class)
            ((YoBoolean) yoVariables.get(keys.keys().get(i))).set((Boolean) newValues.get(i));
      }
   }

   @Override
   public void save()
   {
      // this class is probably gonna get reimplemented
   }

   @Override
   public void load()
   {
      // this class is probably gonna get reimplemented
   }
}