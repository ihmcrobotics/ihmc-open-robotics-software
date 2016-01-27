package us.ihmc.aware.params;

import java.util.HashMap;
import java.util.Map;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class ParameterMapRepository
{
   private final YoVariableRegistry registry;

   private final ParameterMap rootParams;
   private final Map<Class<?>, ParameterMap> repository;

   public ParameterMapRepository(YoVariableRegistry registry)
   {
      this.registry = registry;
      this.rootParams = new ParameterMap(registry, this.getClass());
      this.repository = new HashMap<>();
   }

   public ParameterMap get(Class<?> namespace)
   {
      if (!repository.containsKey(namespace))
      {
         ParameterMap params = new ParameterMap(registry, namespace);
         repository.put(namespace, params);
      }

      return repository.get(namespace);
   }

   public ParameterMap getRootParams()
   {
      return rootParams;
   }
}
