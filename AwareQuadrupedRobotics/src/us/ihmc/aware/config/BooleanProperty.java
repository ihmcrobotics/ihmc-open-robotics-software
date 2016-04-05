package us.ihmc.aware.config;

public class BooleanProperty extends Property
{
   private boolean value;

   protected BooleanProperty(String path, boolean defaultValue)
   {
      super(path);
      this.value = defaultValue;
   }

   public boolean get()
   {
      return value;
   }

   public void set(boolean value)
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

