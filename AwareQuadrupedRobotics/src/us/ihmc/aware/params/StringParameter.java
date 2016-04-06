package us.ihmc.aware.params;

public class StringParameter extends Parameter
{
   private String value;

   StringParameter(String path, String defaultValue)
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
   public boolean tryLoadValue(String value)
   {
      this.value = value;
      return true;
   }

   @Override
   String dumpValue()
   {
      return value;
   }
}

