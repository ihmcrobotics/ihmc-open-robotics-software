package us.ihmc.tools.property;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.HashMap;

public class YoVariableStoredPropertyMap extends HashMap<StoredPropertyKey, YoVariable>
{
   private final StoredPropertySetBasics storedPropertySet;
   private final StoredPropertyKeyList keys;
   private final YoVariableRegistry registry;

   public YoVariableStoredPropertyMap(StoredPropertySetBasics storedPropertySet, StoredPropertyKeyList keys, String registryName)
   {
      this.storedPropertySet = storedPropertySet;
      this.keys = keys;

      registry = new YoVariableRegistry(registryName);

      for (StoredPropertyKey<?> key : keys.keys())
      {
         if (key.getType() == Double.class)
         {
            YoDouble yoDouble = new YoDouble(key.getCamelCasedName(), registry);
            put(key, yoDouble);
            storedPropertySet.addPropertyChangedListener(key, () -> yoDouble.set(storedPropertySet.get((DoubleStoredPropertyKey) key)));
            yoDouble.addVariableChangedListener(v -> storedPropertySet.set((DoubleStoredPropertyKey) key, yoDouble.getValue()));
         }
         else if (key.getType() == Integer.class)
         {
            YoInteger yoInteger = new YoInteger(key.getCamelCasedName(), registry);
            put(key, yoInteger);
            storedPropertySet.addPropertyChangedListener(key, () -> yoInteger.set(storedPropertySet.get((IntegerStoredPropertyKey) key)));
            yoInteger.addVariableChangedListener(v -> storedPropertySet.set((IntegerStoredPropertyKey) key, yoInteger.getValue()));
         }
         else if (key.getType() == Boolean.class)
         {
            YoBoolean yoBoolean = new YoBoolean(key.getCamelCasedName(), registry);
            put(key, yoBoolean);
            storedPropertySet.addPropertyChangedListener(key, () -> yoBoolean.set(storedPropertySet.get((BooleanStoredPropertyKey) key)));
            yoBoolean.addVariableChangedListener(v -> storedPropertySet.set((BooleanStoredPropertyKey) key, yoBoolean.getValue()));
         }
         else
         {
            throw new RuntimeException("Type is not yet implemented.");
         }
      }
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
