package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Optional;

import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.parameters.LongParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterProvider
{
   /**
    * This method will search for an existing parameter and if it exists return it. Otherwise it will create a new one.
    * This is useful when sharing a parameter between modules inside the controller e.g. different robot sides. The class
    * can then request for the parameter to be created in a shared, side independent registry higher up in the tree.
    * <p>
    * The registry that will be searched for the parameter is specified by the method parameters:</br>
    *
    * <pre>
    * {@code
    * -> root
    *  |-> ...
    *  |  |-> targetRegistry
    *  |  |  |-> ...
    *  |  |  |  |-> ...
    *  |  |  |  |  |-> downstreamRegistry
    *  |  |  |-> parameterRegistry
    * }
    * </pre>
    *
    * The registry tree is traversed up starting from the provided downstream registry until the target registry is
    * encountered. This registry will be searched for the child that is the parameter registry. If it does not exist it
    * will be created. Then the parameter registry will be searched for the parameter. If it does not exist it will be
    * created.
    *
    * @throws RuntimeException in case the target registry does not exist or the parameter exists but is not of the
    *    expected type
    * @param targetRegistryName name of the target registry that will be searched for the child parameterRegistry
    * @param parameterRegistryName name of the parameter registry (will be created if not found)
    * @param parameterName name of the parameter (will be created if not found)
    * @param downstreamRegistry some registry that is downstream the target registry in the registry tree
    * @param initialValue the initial value for the parameter in case it needs to be created
    * @return the parameter
    */
   public static DoubleParameter getOrCreateParameter(String targetRegistryName, String parameterRegistryName, String parameterName,
                                                      YoVariableRegistry downstreamRegistry, double initialValue)
   {
      YoVariableRegistry registry = findRegistry(targetRegistryName, parameterRegistryName, downstreamRegistry);
      DoubleParameter parameter = findParameter(registry, parameterName, DoubleParameter.class);

      if (parameter != null)
      {
         return parameter;
      }

      return new DoubleParameter(parameterName, registry, initialValue);
   }

   /**
    * This method will search for an existing parameter and if it exists return it. Otherwise it will create a new one.
    * This is useful when sharing a parameter between modules inside the controller e.g. different robot sides. The class
    * can then request for the parameter to be created in a shared, side independent registry higher up in the tree.
    * <p>
    * The registry that will be searched for the parameter is specified by the method parameters:</br>
    *
    * <pre>
    * {@code
    * -> root
    *  |-> ...
    *  |  |-> targetRegistry
    *  |  |  |-> ...
    *  |  |  |  |-> ...
    *  |  |  |  |  |-> downstreamRegistry
    *  |  |  |-> parameterRegistry
    * }
    * </pre>
    *
    * The registry tree is traversed up starting from the provided downstream registry until the target registry is
    * encountered. This registry will be searched for the child that is the parameter registry. If it does not exist it
    * will be created. Then the parameter registry will be searched for the parameter. If it does not exist it will be
    * created.
    *
    * @throws RuntimeException in case the target registry does not exist or the parameter exists but is not of the
    *    expected type
    * @param targetRegistryName name of the target registry that will be searched for the child parameterRegistry
    * @param parameterRegistryName name of the parameter registry (will be created if not found)
    * @param parameterName name of the parameter (will be created if not found)
    * @param downstreamRegistry some registry that is downstream the target registry in the registry tree
    * @param initialValue the initial value for the parameter in case it needs to be created
    * @return the parameter
    */
   public static BooleanParameter getOrCreateParameter(String targetRegistryName, String parameterRegistryName, String parameterName,
                                                       YoVariableRegistry downstreamRegistry, boolean initialValue)
   {
      YoVariableRegistry registry = findRegistry(targetRegistryName, parameterRegistryName, downstreamRegistry);
      BooleanParameter parameter = findParameter(registry, parameterName, BooleanParameter.class);

      if (parameter != null)
      {
         return parameter;
      }

      return new BooleanParameter(parameterName, registry, initialValue);
   }

   /**
    * This method will search for an existing parameter and if it exists return it. Otherwise it will create a new one.
    * This is useful when sharing a parameter between modules inside the controller e.g. different robot sides. The class
    * can then request for the parameter to be created in a shared, side independent registry higher up in the tree.
    * <p>
    * The registry that will be searched for the parameter is specified by the method parameters:</br>
    *
    * <pre>
    * {@code
    * -> root
    *  |-> ...
    *  |  |-> targetRegistry
    *  |  |  |-> ...
    *  |  |  |  |-> ...
    *  |  |  |  |  |-> downstreamRegistry
    *  |  |  |-> parameterRegistry
    * }
    * </pre>
    *
    * The registry tree is traversed up starting from the provided downstream registry until the target registry is
    * encountered. This registry will be searched for the child that is the parameter registry. If it does not exist it
    * will be created. Then the parameter registry will be searched for the parameter. If it does not exist it will be
    * created.
    *
    * @throws RuntimeException in case the target registry does not exist or the parameter exists but is not of the
    *    expected type
    * @param targetRegistryName name of the target registry that will be searched for the child parameterRegistry
    * @param parameterRegistryName name of the parameter registry (will be created if not found)
    * @param parameterName name of the parameter (will be created if not found)
    * @param downstreamRegistry some registry that is downstream the target registry in the registry tree
    * @param initialValue the initial value for the parameter in case it needs to be created
    * @return the parameter
    */
   public static IntegerParameter getOrCreateParameter(String targetRegistryName, String parameterRegistryName, String parameterName,
                                                       YoVariableRegistry downstreamRegistry, int initialValue)
   {
      YoVariableRegistry registry = findRegistry(targetRegistryName, parameterRegistryName, downstreamRegistry);
      IntegerParameter parameter = findParameter(registry, parameterName, IntegerParameter.class);

      if (parameter != null)
      {
         return parameter;
      }

      return new IntegerParameter(parameterName, registry, initialValue);
   }

   /**
    * This method will search for an existing parameter and if it exists return it. Otherwise it will create a new one.
    * This is useful when sharing a parameter between modules inside the controller e.g. different robot sides. The class
    * can then request for the parameter to be created in a shared, side independent registry higher up in the tree.
    * <p>
    * The registry that will be searched for the parameter is specified by the method parameters:</br>
    *
    * <pre>
    * {@code
    * -> root
    *  |-> ...
    *  |  |-> targetRegistry
    *  |  |  |-> ...
    *  |  |  |  |-> ...
    *  |  |  |  |  |-> downstreamRegistry
    *  |  |  |-> parameterRegistry
    * }
    * </pre>
    *
    * The registry tree is traversed up starting from the provided downstream registry until the target registry is
    * encountered. This registry will be searched for the child that is the parameter registry. If it does not exist it
    * will be created. Then the parameter registry will be searched for the parameter. If it does not exist it will be
    * created.
    *
    * @throws RuntimeException in case the target registry does not exist or the parameter exists but is not of the
    *    expected type
    * @param targetRegistryName name of the target registry that will be searched for the child parameterRegistry
    * @param parameterRegistryName name of the parameter registry (will be created if not found)
    * @param parameterName name of the parameter (will be created if not found)
    * @param downstreamRegistry some registry that is downstream the target registry in the registry tree
    * @param initialValue the initial value for the parameter in case it needs to be created
    * @return the parameter
    */
   public static LongParameter getOrCreateParameter(String targetRegistryName, String parameterRegistryName, String parameterName,
                                                    YoVariableRegistry downstreamRegistry, long initialValue)
   {
      YoVariableRegistry registry = findRegistry(targetRegistryName, parameterRegistryName, downstreamRegistry);
      LongParameter parameter = findParameter(registry, parameterName, LongParameter.class);

      if (parameter != null)
      {
         return parameter;
      }

      return new LongParameter(parameterName, registry, initialValue);
   }

   /**
    * WARNING: This method does not check whether the enum type of an existing parameter matches the requested type.
    * <p>
    * This method will search for an existing parameter and if it exists return it. Otherwise it will create a new one.
    * This is useful when sharing a parameter between modules inside the controller e.g. different robot sides. The class
    * can then request for the parameter to be created in a shared, side independent registry higher up in the tree.
    * <p>
    * The registry that will be searched for the parameter is specified by the method parameters:</br>
    *
    * <pre>
    * {@code
    * -> root
    *  |-> ...
    *  |  |-> targetRegistry
    *  |  |  |-> ...
    *  |  |  |  |-> ...
    *  |  |  |  |  |-> downstreamRegistry
    *  |  |  |-> parameterRegistry
    * }
    * </pre>
    *
    * The registry tree is traversed up starting from the provided downstream registry until the target registry is
    * encountered. This registry will be searched for the child that is the parameter registry. If it does not exist it
    * will be created. Then the parameter registry will be searched for the parameter. If it does not exist it will be
    * created.
    *
    * @throws RuntimeException in case the target registry does not exist or the parameter exists but is not of the
    *    expected type
    * @param <E> the type of enum that this {@code EnumParameter<E>} is for.
    * @param targetRegistryName name of the target registry that will be searched for the child parameterRegistry
    * @param parameterRegistryName name of the parameter registry (will be created if not found)
    * @param parameterName name of the parameter (will be created if not found)
    * @param downstreamRegistry some registry that is downstream the target registry in the registry tree
    * @param enumClass the class of the enum type E in case it needs to be created
    * @param allowNullValues whether null is an acceptable value for this parameter in case it needs to be created
    * @param initialValue the initial value for the parameter in case it needs to be created
    * @return the parameter
    */
   @SuppressWarnings("unchecked")
   public static <E extends Enum<E>> EnumParameter<E> getOrCreateParameter(String targetRegistryName, String parameterRegistryName, String parameterName,
                                                                           YoVariableRegistry downstreamRegistry, Class<E> enumClass, boolean allowNullValues,
                                                                           E initialValue)
   {
      YoVariableRegistry registry = findRegistry(targetRegistryName, parameterRegistryName, downstreamRegistry);
      EnumParameter<E> parameter = findParameter(registry, parameterName, EnumParameter.class);

      if (parameter != null)
      {
         return parameter;
      }

      return new EnumParameter<E>(parameterName, registry, enumClass, allowNullValues, initialValue);
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

   private static YoVariableRegistry findRegistry(String targetRegistryName, String parameterRegistryName, YoVariableRegistry downstreamRegistry)
   {
      YoVariableRegistry registry = downstreamRegistry;
      while (!registry.getName().equals(targetRegistryName))
      {
         registry = registry.getParent();
         if (registry == null)
         {
            throw new RuntimeException("Could not find registry " + targetRegistryName + " as parent of " + downstreamRegistry.getName()
                  + ". Did you add all children yet?");
         }
      }

      Optional<YoVariableRegistry> parameterRegistry = registry.getChildren().stream().filter(r -> r.getName().equals(parameterRegistryName)).findFirst();
      if (parameterRegistry.isPresent())
      {
         return parameterRegistry.get();
      }

      YoVariableRegistry newParameterRegistry = new YoVariableRegistry(parameterRegistryName);
      registry.addChild(newParameterRegistry);
      return newParameterRegistry;
   }
}
