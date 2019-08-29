package us.ihmc.tools.property;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.HashMap;

public class YoVariablesForStoredProperties extends HashMap<StoredPropertyKey, YoVariable>
{
   private final StoredPropertySetBasics storedPropertySet;
   private final StoredPropertyKeyList keys;
   private final YoVariableRegistry registry;

   public YoVariablesForStoredProperties(StoredPropertySetBasics storedPropertySet, StoredPropertyKeyList keys, String registryName)
   {
      this.storedPropertySet = storedPropertySet;
      this.keys = keys;

      registry = new YoVariableRegistry(registryName);

      for (StoredPropertyKey<?> key : keys.keys())
      {
         if (key.getType() == Double.class)
         {
            DoubleStoredPropertyKey castedKey = (DoubleStoredPropertyKey) key;
            YoDouble yoDouble = new YoDouble(key.getCamelCasedName(), registry);
            yoDouble.set(storedPropertySet.get(castedKey), false);
            put(key, yoDouble);
            storedPropertySet.addPropertyChangedListener(key, () -> yoDouble.set(storedPropertySet.get(castedKey)));
            yoDouble.addVariableChangedListener(v -> storedPropertySet.set(castedKey, yoDouble.getValue()));
         }
         else if (key.getType() == Integer.class)
         {
            IntegerStoredPropertyKey castedKey = (IntegerStoredPropertyKey) key;
            YoInteger yoInteger = new YoInteger(key.getCamelCasedName(), registry);
            yoInteger.set(storedPropertySet.get(castedKey), false);
            put(key, yoInteger);
            storedPropertySet.addPropertyChangedListener(key, () -> yoInteger.set(storedPropertySet.get(castedKey)));
            yoInteger.addVariableChangedListener(v -> storedPropertySet.set(castedKey, yoInteger.getValue()));
         }
         else if (key.getType() == Boolean.class)
         {
            BooleanStoredPropertyKey castedKey = (BooleanStoredPropertyKey) key;
            YoBoolean yoBoolean = new YoBoolean(key.getCamelCasedName(), registry);
            yoBoolean.set(storedPropertySet.get(castedKey), false);
            put(key, yoBoolean);
            storedPropertySet.addPropertyChangedListener(key, () -> yoBoolean.set(storedPropertySet.get(castedKey)));
            yoBoolean.addVariableChangedListener(v -> storedPropertySet.set(castedKey, yoBoolean.getValue()));
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
