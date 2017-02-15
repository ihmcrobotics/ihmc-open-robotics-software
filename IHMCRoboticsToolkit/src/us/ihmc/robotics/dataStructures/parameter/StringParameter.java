package us.ihmc.robotics.dataStructures.parameter;

public class StringParameter extends Parameter
{
   private String value;

   private StringParameter() // no-arg for serialization
   {
      super(null);
      this.value = null;
   }

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
      super.notifyChangeListeners();
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

