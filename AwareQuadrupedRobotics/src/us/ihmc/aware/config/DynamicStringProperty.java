package us.ihmc.aware.config;

public class DynamicStringProperty extends DynamicProperty
{
   private String value;

   public DynamicStringProperty(DynamicPropertyRegistry registry, String path, String defaultValue)
   {
      super(registry, path);
      this.value = defaultValue;
   }

   public String get()
   {
      return value;
   }

   public void set(String value)
   {
      this.value = value;
      notifyChangedListeners();
   }

   @Override
   public String dump()
   {
      return getPath() + "=" + value;
   }
}

