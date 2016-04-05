package us.ihmc.aware.config;

public class PropertyFactory
{
   private final PropertyRegistry registry;
   private final String namespace;

   public PropertyFactory(PropertyRegistry registry, String namespace)
   {
      this.registry = registry;
      this.namespace = namespace;
   }

   public BooleanProperty createBoolean(String name, boolean defaultValue)
   {
      BooleanProperty property = new BooleanProperty(namespace + "." + name, defaultValue);
      registry.register(property);
      return property;
   }

   public DoubleProperty createDouble(String name, double defaultValue)
   {
      DoubleProperty property = new DoubleProperty(namespace + "." + name, defaultValue);
      registry.register(property);
      return property;
   }

   public DoubleArrayProperty createDoubleArray(String name, double... defaultValue)
   {
      DoubleArrayProperty property = new DoubleArrayProperty(namespace + "." + name, defaultValue);
      registry.register(property);
      return property;
   }

   public StringProperty createString(String name, String defaultValue)
   {
      StringProperty property = new StringProperty(namespace + "." + name, defaultValue);
      registry.register(property);
      return property;
   }
}
