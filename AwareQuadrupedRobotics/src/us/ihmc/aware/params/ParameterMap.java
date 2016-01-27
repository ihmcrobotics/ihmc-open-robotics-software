package us.ihmc.aware.params;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/**
 * A namespaces mapping of {@link String} keys to {@link DoubleYoVariable} values.
 */
public class ParameterMap
{
   private final YoVariableRegistry registry;
   private final Class<?> namespace;

   private final Map<String, DoubleYoVariable[]> map = new HashMap<>();

   protected ParameterMap(YoVariableRegistry registry, Class<?> namespace)
   {
      this.registry = registry;
      this.namespace = namespace;
   }

   /**
    * Insert a parameter of arbitrary length into the map.
    *
    * @param name   the base parameter name
    * @param values the parameter values
    */
   public void insert(String name, double... values)
   {
      DoubleYoVariable[] variables = new DoubleYoVariable[values.length];
      for (int i = 0; i < values.length; i++)
      {
         // Create variable names like "myVariable0", "myVariable1", etc.
         final String variableName = "parameter_" + namespace.getSimpleName() + "_" + name + i;

         variables[i] = new DoubleYoVariable(variableName, registry);
         variables[i].set(values[i]);
      }

      map.put(name, variables);
   }

   // TODO: Need to know the array length at insertion time, so this signature doesn't make sense?
   //   public void setDefault(String name, int idx, double value)
   //   {
   //      setDefault(name, new double[] { value });
   //   }

   /**
    * Establishes a default value for the given parameter base name. If the key is not present in the map, then it will
    * be inserted with the given value. If it is present then no action will be taken.
    *
    * @param name   the base parameter name
    * @param values the parameter values
    */
   public void setDefault(String name, double... values)
   {
      if (!contains(name))
      {
         insert(name, values);
      }
   }

   /**
    * @param name the base parameter name
    * @return the first entry stored at the given parameter
    */
   public double get(String name)
   {
      return get(name, 0);
   }

   public void get(String name, double[] values)
   {
      for (int i = 0; i < values.length; i++)
      {
         values[i] = get(name, i);
      }
   }

   public double get(String name, int idx)
   {
      verifyContains(name, idx);

      DoubleYoVariable[] variables = map.get(name);
      return variables[idx].getDoubleValue();
   }

   public void set(String name, double... values)
   {
      for (int i = 0; i < values.length; i++)
      {
         set(name, values[i], i);
      }
   }

   public void set(String name, double value, int idx)
   {
      verifyContains(name, idx);

      DoubleYoVariable[] variables = map.get(name);
      variables[idx].set(value);
   }

   public boolean contains(String name)
   {
      return contains(name, 0);
   }

   public boolean contains(String name, int idx)
   {
      DoubleYoVariable[] variables = map.get(name);

      return variables != null && variables.length > idx;
   }

   /**
    * Ensures that the given parameter exists in the map and throws an error if it does not.
    *
    * @param name
    * @param idx
    * @throws ParameterDoesNotExistException     if no indices of the parameter exist
    * @throws ParameterIndexOutOfBoundsException if only the given index does not exist
    */
   private void verifyContains(String name, int idx)
   {
      if (!contains(name))
      {
         throw new ParameterDoesNotExistException(name);
      }

      if (!contains(name, idx))
      {
         throw new ParameterIndexOutOfBoundsException(name, idx);
      }
   }
}
