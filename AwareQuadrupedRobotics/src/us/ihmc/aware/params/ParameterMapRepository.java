package us.ihmc.aware.params;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

/**
 * A collection of {@link ParameterMap}s indexed by their respective controller classes.
 * <p/>
 * Each controller is meant to have its own mapping of parameters. This parameter map can be accessed with {@link
 * #get(Class)}.
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

   /**
    * @return the single root parameter map for this repository
    */
   public ParameterMap getRootParams()
   {
      return null;  // Temporary comment out since not compiling in order to fix Bamboo. J.Pratt March 17, 2016
//      return rootParams;
   }
}
