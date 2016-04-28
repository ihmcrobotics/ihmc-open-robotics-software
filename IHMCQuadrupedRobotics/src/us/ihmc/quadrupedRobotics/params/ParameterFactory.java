package us.ihmc.quadrupedRobotics.params;

/**
 * A factory for creating and registering parameters and their default values.
 * <p/>
 * This class is the front-end for declaring parameters in user classes.
 * <p/>
 * Example usage:
 * <pre>
 * class MyClass {
 *    ParameterFactory parameterFactory = new ParameterFactory(getClass());
 *    DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 2);
 * }
 * </pre>
 */
public class ParameterFactory
{
   private final String namespace;

   /**
    * Create a new parameter factory. Each namespace (class) should create its own
    *
    * @param namespace
    */
   public ParameterFactory(Class<?> namespace)
   {
      this.namespace = namespace.getName();
   }

   public BooleanParameter createBoolean(String name, boolean defaultValue)
   {
      BooleanParameter parameter = new BooleanParameter(namespace + "." + name, defaultValue);
      register(parameter);
      return parameter;
   }

   public DoubleParameter createDouble(String name, double defaultValue)
   {
      DoubleParameter parameter = new DoubleParameter(namespace + "." + name, defaultValue);
      register(parameter);
      return parameter;
   }

   public DoubleArrayParameter createDoubleArray(String name, double... defaultValue)
   {
      DoubleArrayParameter parameter = new DoubleArrayParameter(namespace + "." + name, defaultValue);
      register(parameter);
      return parameter;
   }

   public StringParameter createString(String name, String defaultValue)
   {
      StringParameter parameter = new StringParameter(namespace + "." + name, defaultValue);
      register(parameter);
      return parameter;
   }

   private static void register(Parameter parameter)
   {
      ParameterRegistry.getInstance().register(parameter);
   }
}
