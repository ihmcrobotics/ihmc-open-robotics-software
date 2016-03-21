package us.ihmc.aware.params;

import java.io.Reader;
import java.io.Writer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.commons.lang3.ArrayUtils;
import org.yaml.snakeyaml.Yaml;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

/**
 * A collection of {@link ParameterMap}s indexed by their respective controller classes.
 * <p/>
 * Each controller is meant to have its own mapping of parameters. This parameter map can be accessed with {@link #get(Class)}.
 */
public class ParameterMapRepository
{
   private final YoVariableRegistry registry;

   private final Map<Class<?>, ParameterMap> repository;

   /**
    * @param registry the registry in which to store all of the parameters
    */
   public ParameterMapRepository(YoVariableRegistry registry)
   {
      this.registry = registry;
      this.repository = new HashMap<>();
   }

   /**
    * Fetch or create the map for the given namespace.
    *
    * @param namespace the desired namespace
    * @return a new parameter map if it was not yet registered, or the previously registered one
    */
   public ParameterMap get(Class<?> namespace)
   {
      if (!repository.containsKey(namespace))
      {
         ParameterMap params = new ParameterMap(registry, namespace);
         repository.put(namespace, params);
      }

      return repository.get(namespace);
   }

   @SuppressWarnings("unchecked")
   public void load(Reader reader)
   {
      // TODO: Lots of unchecked casts here that depend on the formatting of the input YAML file. Is it possible to do some more checks to ensure consistent
      // input format before throwing an exception on a bad cast?

      Map paramsByClass = (Map) new Yaml().load(reader);
      for (String className : (Set<String>) paramsByClass.keySet())
      {
         try
         {
            Class<?> clazz = Class.forName(className);
            ParameterMap paramMap = get(clazz);

            Map<String, List<Double>> paramsByKey = (Map<String, List<Double>>) paramsByClass.get(className);
            for (Map.Entry<String, List<Double>> entry : paramsByKey.entrySet())
            {
               String paramName = entry.getKey();
               double[] paramValues = objectListToPrimitiveArray(entry.getValue());

               // Don't try to set parameters that don't exist in the map.
               if (paramMap.contains(paramName))
               {
                  paramMap.set(paramName, paramValues);
               }
               else
               {
                  System.err.println("Parameter file contains " + className + "." + paramName + " which does not exist in the map!");
               }
            }
         }
         catch (ClassNotFoundException e)
         {
            System.err.print("Can't load parameters for " + className + ". Does the class still exist?");
            e.printStackTrace();
         }
      }
   }

   public void dump(Writer writer)
   {
      Map<String, Map<String, double[]>> dumped = new HashMap<>();
      for (Map.Entry<Class<?>, ParameterMap> entry : repository.entrySet())
      {
         String className = entry.getKey().getName();
         ParameterMap params = entry.getValue();

         dumped.put(className, params.dump());
      }

      new Yaml().dump(dumped, writer);
   }

   private double[] objectListToPrimitiveArray(List<Double> objects)
   {
      Double[] objectArray = new Double[objects.size()];
      objects.toArray(objectArray);
      return ArrayUtils.toPrimitive(objectArray);
   }
}
