package us.ihmc.aware.config;

public class ParameterFactory
{
   private final String namespace;

   public ParameterFactory(String namespace)
   {
      this.namespace = namespace;
   }

   public BooleanParameter createBoolean(String name, boolean defaultValue)
   {
      BooleanParameter property = new BooleanParameter(namespace + "." + name, defaultValue);
      ParameterRegistry.getInstance().register(property);
      return property;
   }

   public DoubleParameter createDouble(String name, double defaultValue)
   {
      DoubleParameter property = new DoubleParameter(namespace + "." + name, defaultValue);
      ParameterRegistry.getInstance().register(property);
      return property;
   }

   public DoubleArrayParameter createDoubleArray(String name, double... defaultValue)
   {
      DoubleArrayParameter property = new DoubleArrayParameter(namespace + "." + name, defaultValue);
      ParameterRegistry.getInstance().register(property);
      return property;
   }

   public StringParameter createString(String name, String defaultValue)
   {
      StringParameter property = new StringParameter(namespace + "." + name, defaultValue);
      ParameterRegistry.getInstance().register(property);
      return property;
   }
}
