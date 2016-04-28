package us.ihmc.quadrupedRobotics.params;

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

