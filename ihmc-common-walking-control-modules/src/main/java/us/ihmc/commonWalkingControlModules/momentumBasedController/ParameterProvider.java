package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Optional;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterProvider
{
   public static DoubleParameter getParameter(String registryName, String parameterName, YoVariableRegistry downstreamRegistry, double initialValue)
   {
      YoVariableRegistry registry = findRegistry(registryName, downstreamRegistry);
      DoubleParameter parameter = findParameter(registry, parameterName, DoubleParameter.class);
      if (parameter != null)
      {
         return parameter;
      }
      return new DoubleParameter(parameterName, registry, initialValue);
   }

   private static <T extends YoParameter<?>> T findParameter(YoVariableRegistry registry, String parameterName, Class<T> clazz)
   {
      Optional<YoParameter<?>> parameter = registry.getAllParameters().stream().filter(p -> p.getName().equals(parameterName)).findFirst();

      if (parameter.isPresent())
      {
         if (parameter.get().getClass() == clazz)
         {
            return clazz.cast(parameter.get());
         }
         else
         {
            throw new RuntimeException("Found parameter " + parameterName + " in " + registry.getName() + ". Expected " + clazz.getSimpleName() + " but found "
                  + parameter.get().getClass().getSimpleName() + ".");
         }
      }

      return null;
   }

   private static YoVariableRegistry findRegistry(String registryName, YoVariableRegistry downstreamRegistry)
   {
      YoVariableRegistry registry = downstreamRegistry;
      while (!registry.getName().equals(registryName))
      {
         registry = registry.getParent();
         if (registry == null)
         {
            throw new RuntimeException("Could not find registry " + registryName + " as parent of " + downstreamRegistry.getName()
                  + ". Did you add all children yet?");
         }
      }
      return registry;
   }
}
