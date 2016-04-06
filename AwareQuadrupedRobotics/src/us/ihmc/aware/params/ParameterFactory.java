package us.ihmc.aware.params;

public class ParameterFactory
{
   private final String namespace;

   public ParameterFactory(String namespace)
   {
      this.namespace = namespace;
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

   private void register(Parameter parameter)
   {
      ParameterRegistry.getInstance().register(parameter);
   }
}
