package us.ihmc.aware.config;

public class StringProperty extends Property
{
   private String value;

   protected StringProperty(String path, String defaultValue)
   {
      super(path);
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

